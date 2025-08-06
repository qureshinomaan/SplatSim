from copy import deepcopy
from torch.functional import norm
from torch import nn

from pyrl.utils.meta import ConfigDict
from pyrl.utils.torch import ExtendedSequential

from ..builder import NETWORK, build_all
from .weight_init import kaiming_init, constant_init
from .nn_layer import INPLACE_ACTIVATIONS, need_bias, set_norm_input


class BasicBlock(ExtendedSequential):
    def __init__(
        self,
        nn_cfg,
        norm_cfg=None,
        act_cfg=None,
        bias="auto",
        inplace=True,
        with_spectral_norm=False,
        order=("dense", "norm", "act"),
        index="",
        use_default_init=False,
        **extra_nn_cfg,
    ):
        super(BasicBlock, self).__init__()
        # dense here is conv or linear
        assert nn_cfg is None or isinstance(nn_cfg, dict)
        assert norm_cfg is None or isinstance(norm_cfg, dict)
        assert act_cfg is None or isinstance(act_cfg, dict)
        assert isinstance(order, tuple) and len(order) == 3
        assert set(order) == set(["dense", "norm", "act"]), f"{order}"
        # assert nn_cfg.get(bias, True) or bias == 'auto', f"{nn_cfg.get(bias, None), bias}"

        self.nn_cfg = nn_cfg
        self.norm_cfg = norm_cfg
        self.act_cfg = act_cfg

        self.with_norm = norm_cfg is not None
        self.with_activation = act_cfg is not None
        self.with_spectral_norm = with_spectral_norm

        norm_follow_dense = order.index("norm") > order.index("dense")
        nn_cfg.update(extra_nn_cfg)
        if nn_cfg.get("bias", bias) == "auto":
            nn_cfg["bias"] = need_bias(norm_cfg)
        nn_cfg.setdefault("bias", bias)

        in_size, out_size = None, None
        for name in ["features", "channels"]:
            in_size_tmp = nn_cfg.get(f"in_{name}", None)
            out_size_tmp = nn_cfg.get(f"out_{name}", None)
            assert (in_size_tmp is None) == (out_size_tmp is None), "in and out must be specified at the same time!"
            if in_size_tmp is not None:
                in_size, out_size = in_size_tmp, out_size_tmp
                break
        assert in_size is not None and out_size is not None, f"{nn_cfg} is not correct!"

        for name in order:
            if name == "dense" and nn_cfg is not None:
                dense_type = nn_cfg.get("type", None)
                assert dense_type is not None and dense_type
                nn_cfg = nn_cfg.copy()

                if "conv" in dense_type.lower():
                    # Padding happen before convolution

                    official_conv_padding_mode = ["zeros", "reflect", "replicate", "circular"]  # Pytorch >= 1.7.1
                    padding_cfg = nn_cfg.pop("padding_cfg", None)
                    padding_mode = nn_cfg.get("padding_mode", None)
                    assert not (padding_cfg is None) or (padding_mode is None), "We only need one of padding_cfg and padding_mode"
                    if padding_cfg is not None:
                        padding_mode = padding_cfg.get("type", None)
                        if padding_mode is not None:
                            if padding_mode not in official_conv_padding_mode:
                                pad_cfg = dict(type=padding_mode)
                                self.add_module("padding", build_all(pad_cfg))
                            else:
                                nn_cfg["padding_mode"] = padding_mode
                    elif padding_mode is not None:
                        assert padding_mode in official_conv_padding_mode

                    layer = build_all(nn_cfg)
                    if self.with_spectral_norm:
                        layer = nn.utils.spectral_norm(layer)
                    self.add_module(f"conv{index}", layer)
                elif "linear" in dense_type.lower():
                    self.add_module(f"linear{index}", build_all(nn_cfg))
                else:
                    raise NotImplementedError(f"{dense_type} is not supported!")
            elif name == "act" and act_cfg is not None:
                act_cfg = act_cfg.copy()
                if act_cfg["type"] in INPLACE_ACTIVATIONS:
                    act_cfg.setdefault("inplace", inplace)
                self.add_module(f"act{index}", build_all(act_cfg))
            elif name == "norm" and norm_cfg is not None:
                norm_channels = out_size if norm_follow_dense else in_size
                norm = build_all(set_norm_input(norm_cfg, norm_channels))
                self.add_module(f"norm{index}", norm)
        if use_default_init:
            self.reset_parameters()

    def reset_parameters(self):
        # 1. It is mainly for customized conv layers with their own initialization manners by calling their own
        #    ``init_weights()``, and we do not want ConvModule to override the initialization.
        # 2. For customized conv layers without their own initialization manners (that is, they don't have their own
        #   ``init_weights()``) and PyTorch's conv layers, they will be initialized by this method with default
        #   ``kaiming_init``.
        # Note: For PyTorch's conv layers, they will be overwritten by our initialization implementation using
        #     default ``kaiming_init``.
        for name, module in self.named_modules():
            if name in ["linear", "conv"]:
                if not hasattr(module, "reset_parameters"):
                    if self.with_activation and self.act_cfg["type"] == "LeakyReLU":
                        nonlinearity, a = "leaky_relu", self.act_cfg.get("negative_slope", 0.01)
                    elif self.act_cfg["type"] == "ReLU":
                        nonlinearity, a = "relu", 0
                    else:
                        nonlinearity = a = None
                    if nonlinearity is not None:
                        kaiming_init(module, a=a, nonlinearity=nonlinearity)
            elif name == "norm":
                constant_init(norm, 1, bias=0)


@NETWORK.register_module()
class FlexibleBasicBlock(BasicBlock):
    # The order of the operation is dynamic
    def forward(self, feature, activate=True, norm=True):
        for name, module in self.named_modules():
            if (name == "act" and not activate) or (name == "norm" and not norm):
                feature = module(feature)
        return feature


@NETWORK.register_module()
class ConvModule(BasicBlock):
    def __init__(self, in_channels, out_channels, kernel_size=1, nn_cfg=None, **kwargs):
        nn_cfg = deepcopy(nn_cfg) if nn_cfg is not None else ConfigDict(type="Conv2d")
        if "type" not in nn_cfg:
            nn_cfg["type"] = "Conv2d"
        nn_cfg["in_channels"] = in_channels
        nn_cfg["out_channels"] = out_channels
        nn_cfg["kernel_size"] = kernel_size
        super(ConvModule, self).__init__(nn_cfg, **kwargs)


@NETWORK.register_module()
class LinearModule(BasicBlock):
    def __init__(self, in_channels, out_channels, nn_cfg=None, **kwargs):
        nn_cfg = deepcopy(nn_cfg) if nn_cfg is not None else ConfigDict(type="Linear")
        nn_cfg["in_features"] = in_channels
        nn_cfg["out_features"] = out_channels
        super(LinearModule, self).__init__(nn_cfg, **kwargs)

