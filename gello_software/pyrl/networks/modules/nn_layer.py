from copy import deepcopy

import torch
import torch.nn as nn
import torch.nn.functional as F
import math

from torch import Tensor
from torch.nn import Parameter, LayerNorm

from ..builder import NETWORK
from pyrl.utils.data import is_num, is_seq_of


def register_torch_nn_module():
    for name, module in nn.__dict__.items():
        if isinstance(module, type):
            NETWORK.register_module(name, module=module)
    alias = {
        "Conv": nn.Conv2d,
        "Deconv": nn.ConvTranspose2d,
        "BN": nn.BatchNorm2d,
        "SyncBN": nn.SyncBatchNorm,
        "GN": nn.GroupNorm,
        "LN": nn.LayerNorm,
        "LRN": nn.LocalResponseNorm,
    }
    for dim in [1, 2, 3]:
        alias[f"BN{dim}d"] = getattr(nn, f"BatchNorm{dim}d")
        alias[f"IN{dim}d"] = getattr(nn, f"InstanceNorm{dim}d")
        alias[f"LazyBN{dim}d"] = getattr(nn, f"LazyBatchNorm{dim}d")
        alias[f"LazyIN{dim}d"] = getattr(nn, f"LazyInstanceNorm{dim}d")

    for name, module in alias.items():
        NETWORK.register_module(name, module=module)


register_torch_nn_module()


""" Activation layers  """


@NETWORK.register_module(name="Clip")
@NETWORK.register_module()
class Clamp(nn.Module):
    def __init__(self, min=-1.0, max=1.0):
        super(Clamp, self).__init__()
        self.min = min
        self.max = max

    def forward(self, x):
        return torch.clamp(x, min=self.min, max=self.max)


try:
    import torchsparse.nn as spnn

    NETWORK.register_module(name="SparseReLU", module=spnn.ReLU)
    NETWORK.register_module(name="SparseLeakyReLU", module=spnn.LeakyReLU)
except ImportError as e:
    print("torchsparse is not installed correctly!")
    print(e)


INPLACE_ACTIVATIONS = ["ELU", "Hardsigmoid", "Hardtanh", "Hardswish", "ReLU", "LeakyReLU", "ReLU6", "RReLU", "SELU", "CELU", "SiLU", "Threshold"]


""" Network layers  """


@NETWORK.register_module()
class Conv2dAdaptivePadding(nn.Conv2d):
    """
    Implementation of 2D convolution in tensorflow with `padding` as "same",
    which applies padding to input so that input image gets fully covered by filter and stride you specified.
    For example:
        With stride 1, this will ensure that output image size is same as input.
        With stride 2, output dimensions will be half.
    Args:
        in_channels (int): Number of channels in the input image
        out_channels (int): Number of channels produced by the convolution
        kernel_size (int or tuple): Size of the convolving kernel
        stride (int or tuple, optional): Stride of the convolution. Default: 1
        padding (int or tuple, optional): Zero-padding added to both sides of the input. Default: 0
        dilation (int or tuple, optional): Spacing between kernel elements. Default: 1
        groups (int, optional): Number of blocked connections from input channels to output channels. Default: 1
        bias (bool, optional): If ``True``, adds a learnable bias to the output. Default: ``True``
    """

    def __init__(self, in_channels, out_channels, kernel_size, stride=1, padding=0, dilation=1, groups=1, bias=True):
        super().__init__(in_channels, out_channels, kernel_size, stride, padding, dilation, groups, bias)

    def forward(self, x):
        img_h, img_w = x.size()[-2:]
        kernel_h, kernel_w = self.weight.size()[-2:]
        stride_h, stride_w = self.stride
        output_h = math.ceil(img_h / stride_h)
        output_w = math.ceil(img_w / stride_w)
        pad_h = max((output_h - 1) * self.stride[0] + (kernel_h - 1) * self.dilation[0] + 1 - img_h, 0)
        pad_w = max((output_w - 1) * self.stride[1] + (kernel_w - 1) * self.dilation[1] + 1 - img_w, 0)
        if pad_h > 0 or pad_w > 0:
            x = F.pad(x, [pad_w // 2, pad_w - pad_w // 2, pad_h // 2, pad_h - pad_h // 2])
        return F.conv2d(x, self.weight, self.bias, self.stride, self.padding, self.dilation, self.groups)


@NETWORK.register_module()
class EnsembleLinear(nn.Module):
    """EnsembleLinear
    Refer the linear module definition: https://pytorch.org/docs/stable/_modules/torch/nn/modules/linear.html#Linear
    """

    __constants__ = ["in_features", "out_features", "num_module"]
    in_features: int
    out_features: int
    num_modules: int
    weight: Tensor

    def __init__(self, in_features: int, out_features: int, num_modules: int = 1, bias: bool = True, device=None, dtype=None) -> None:
        factory_kwargs = {"device": device, "dtype": dtype}
        super(EnsembleLinear, self).__init__()
        self.in_features = in_features
        self.out_features = out_features
        self.num_modules = num_modules
        self.weight = Parameter(torch.zeros((num_modules, out_features, in_features), **factory_kwargs))
        if bias:
            self.bias = Parameter(torch.zeros(num_modules, out_features, **factory_kwargs))
        else:
            self.register_parameter("bias", None)
        self.reset_parameters()

    def reset_parameters(self) -> None:
        import torch.nn.init as init

        init.kaiming_uniform_(self.weight, a=math.sqrt(5))
        if self.bias is not None:
            fan_in, _ = init._calculate_fan_in_and_fan_out(self.weight.data[0])
            bound = 1 / math.sqrt(fan_in) if fan_in > 0 else 0
            init.uniform_(self.bias, -bound, bound)

    def forward(self, input: Tensor) -> Tensor:
        ret = torch.einsum("bki,koi->bko", input, self.weight)
        if self.bias is not None:
            ret = ret + self.bias
        return ret

    def extra_repr(self) -> str:
        return "num_modules={}, in_features={}, out_features={}, bias={}".format(
            self.num_modules, self.in_features, self.out_features, self.bias is not None
        )


def _pad_parameters(conv_in, *args):
    if is_num(conv_in):
        return list(args)
    ret = []
    for arg in args:
        ret.append([arg] * len(conv_in))
    return ret


def conv_out_shape(conv_in, padding, kernel_size, stride):
    if is_num(conv_in):
        return int((conv_in + 2.0 * padding - (kernel_size - 1.0) - 1.0) / stride + 1.0)
    else:
        padding, kernel_size, stride = _pad_parameters(conv_in, padding, kernel_size, stride)
        return tuple(conv_out_shape(conv_in[i], padding[i], kernel_size[i], stride[i]) for i in range(len(conv_in)))


def padding_shape(conv_in, conv_out, padding, kernel_size, stride):
    if is_num(conv_in):
        return conv_in - (conv_out - 1) * stride + 2 * padding - (kernel_size - 1) - 1
    else:
        assert is_seq_of(conv_in) and is_seq_of(conv_out) and len(conv_in) == len(conv_out)
        padding, kernel_size, stride = _pad_parameters(conv_in, padding, kernel_size, stride)
        return tuple(padding_shape(conv_in[i], conv_out[i], padding[i], kernel_size[i], stride[i]) for i in range(len(conv_in)))
    

""" Normalization layers  """

"""
NOTE:
Different implementations of LayerNorm can have significant difference in speed:
https://github.com/pytorch/pytorch/issues/76012

nn.LayerNorm can be much slower than the custom LN version for the ConvNext model due to necessary 
permutation before / after the LN operation
"""


@NETWORK.register_module("LNkd")
class LayerNormkD(LayerNorm):
    r"""Original implementation in PyTorch is not friendly for CNN which has channels_first manner.
    LayerNorm for CNN (1D, 2D, 3D)
    1D: [B, C, N]
    2D: [B, C, W, H]
    3D: [B, C, X, Y, Z]
    Modified from https://github.com/facebookresearch/ConvNeXt/blob/a9c608745273fa8419fe9e8ae3ce5bf8d32009dd/models/convnext.py
    """

    def __init__(self, num_features, *args, dim=1, data_format="channels_first", **kwargs):
        super(LayerNormkD, self).__init__(num_features, *args, **kwargs)
        assert data_format in ["channels_first", "channels_last"]
        self.dim = dim
        self.index_to_cl = [0] + list(range(2, 2 + self.dim)) + [1]
        self.index_to_cf = [0, 1 + self.dim] + list(range(1, 1 + self.dim))
        self.data_format = data_format

    def forward(self, inputs):
        assert inputs.ndim == self.dim + 2 or (self.dim == 1 and inputs.ndim == 2)
        if self.data_format == "channels_last":
            return super(LayerNormkD, self).forward(inputs)
        else:
            if inputs.ndim > 2:
                inputs = inputs.permute(self.index_to_cl).contiguous()
            ret = super(LayerNormkD, self).forward(inputs)
            if inputs.ndim > 2:
                ret = ret.permute(self.index_to_cf).contiguous()
            return ret


@NETWORK.register_module("LN1d")
class LayerNorm1D(LayerNormkD):
    def __init__(self, num_features, *args, data_format="channels_first", **kwargs):
        super(LayerNorm1D, self).__init__(num_features, *args, **kwargs, data_format=data_format, dim=1)


@NETWORK.register_module("LN2d")
class LayerNorm2D(LayerNormkD):
    def __init__(self, num_features, *args, data_format="channels_first", **kwargs):
        super(LayerNorm2D, self).__init__(num_features, *args, **kwargs, data_format=data_format, dim=2)


@NETWORK.register_module("LN3d")
class LayerNorm3D(LayerNormkD):
    def __init__(self, num_features, *args, data_format="channels_first", **kwargs):
        super(LayerNorm3D, self).__init__(num_features, *args, **kwargs, data_format=data_format, dim=3)


def need_bias(norm_cfg):
    if norm_cfg is None:
        return True
    if "BN" in norm_cfg["type"] or "GN" in norm_cfg["type"]:
        affine = norm_cfg.get("affine", True)
    elif "LN" in norm_cfg["type"]:
        affine = norm_cfg.get("elementwise_affine", True)
    elif "IN" in norm_cfg["type"]:
        affine = norm_cfg.get("affine", False)
    elif "LRN" in norm_cfg["type"]:
        affine = False
    else:
        raise TypeError(norm_cfg["type"])
    return not affine


def is_ln(norm_cfg):
    if norm_cfg is not None:
        norm_type = norm_cfg.get("type", None)
        if norm_type is not None and ("LN" in norm_type or "Layer" in norm_type):
            return True
    return False


def set_norm_input(norm_cfg, input_size):
    norm_cfg = deepcopy(norm_cfg)
    norm_class = NETWORK.get(norm_cfg["type"])
    channel_name = norm_class.__init__.__code__.co_varnames[1]
    norm_cfg[channel_name] = input_size
    return norm_cfg
    