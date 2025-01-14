import torch, numpy as np, torch.nn as nn, copy
import torch.nn.functional as F
from torch.autograd import Function
from torch.nn.parallel import DistributedDataParallel
from pyrl.utils.data import is_num, regex_match


@torch.no_grad()
def get_flat_params(model, trainable=False):
    params = []
    for param in model.parameters():
        if trainable and not param.requires_grad:
            continue
        params.append(param.data.view(-1))

    flat_params = torch.cat(params)
    return flat_params


@torch.no_grad()
def set_flat_params(model, flat_params, trainable=False):
    prev_ind = 0
    for param in model.parameters():
        if trainable and not param.requires_grad:
            continue
        flat_size = int(param.numel())
        param.data.copy_(flat_params[prev_ind : prev_ind + flat_size].view(param.size()))
        prev_ind += flat_size


@torch.no_grad()
def get_flat_grads(net, grad_grad=False):
    grads = []
    for param in net.parameters():
        if not param.requires_grad:
            continue
        grad = param.grad.grad if grad_grad else param.grad
        if grad is None:
            grad = torch.zeros_like(param.data)
        else:
            grad = grad.data
        grads.append(grad.view(-1))
    flat_grad = torch.cat(grads)
    return flat_grad


@torch.no_grad()
def set_flat_grads(net, flat_grads):
    prev_ind = 0
    for param in net.parameters():
        if not param.requires_grad:
            continue
        flat_size = int(param.numel())
        if param.grad is not None:
            param.grad.data.copy_(flat_grads[prev_ind : prev_ind + flat_size].view(param.size()))
        prev_ind += flat_size


@torch.no_grad()
def soft_update(target, source, tau):
    if isinstance(target, nn.Parameter):
        if id(target) != id(source):
            # print(target.shape)
            target.data.copy_(target.data * (1.0 - tau) + source.data * tau)
    elif is_num(tau):
        for target_param, param in zip(target.parameters(), source.parameters()):
            soft_update(target_param, param, tau)
    else:
        assert isinstance(tau, dict), f"tau should be a number or a dict, but the type of tau is {type(tau)}."
        assert "default" in tau, f"The dict needs key default! You dict contains keys: {list(tau.keys())}"
        tau = copy.deepcopy(tau)
        default = tau.pop("default")
        source_param_dict = dict(source.named_parameters())
        target_param_dict = dict(target.named_parameters())
        target_is_ddp, source_is_ddp = isinstance(target, DistributedDataParallel), isinstance(source, DistributedDataParallel)
        for name in source_param_dict:
            if target_is_ddp == source_is_ddp:
                target_name = name
            elif not target_is_ddp and source_is_ddp:
                target_name = name.strip("module.")
            elif target_is_ddp and not source_is_ddp:
                target_name = "module." + name

            target_param, param = target_param_dict[target_name], source_param_dict[name]
            tau_i = default
            for pattern, value in tau.items():
                if regex_match(name, pattern):
                    tau_i = value
                    break
            soft_update(target_param, param, tau_i)


@torch.no_grad()
def hard_update(target, source):
    if isinstance(target, nn.Parameter):
        if id(target) != id(source):
            target.data.copy_(source.data)
    else:
        for target_param, param in zip(target.parameters(), source.parameters()):
            hard_update(target_param, param)


def batch_random_perm(batch_size, num_features, mask=None, device="cuda"):
    if mask is None:
        return torch.rand(batch_size, num_features, device=device).argsort(dim=-1)
    else:
        assert mask.ndim == 2 and mask.shape[0] == batch_size and mask.shape[1] == num_features
        return (torch.rand(batch_size, num_features, device=device) * mask).argsort(dim=-1)


def masked_average(x, axis, mask=None, keepdim=False):
    if mask is None:
        return torch.mean(x, dim=axis, keepdim=keepdim)
    else:
        return torch.sum(x * mask, dim=axis, keepdim=keepdim) / (torch.sum(mask, dim=axis, keepdim=keepdim) + 1e-6)


def masked_max(x, axis, mask=None, keepdim=False, empty_value=0):
    if mask is None:
        return torch.max(x, dim=axis, keepdim=keepdim).values
    else:
        value_with_inf = torch.max(x * mask + -1e18 * (1 - mask), dim=axis, keepdim=keepdim).values
        # The masks are all zero will cause inf
        value = torch.where(value_with_inf > -1e17, value_with_inf, torch.ones_like(value_with_inf) * empty_value)
        return value


class AvgGrad(Function):
    @staticmethod
    def forward(ctx, tensor, num):
        ctx.num = num
        return tensor

    @staticmethod
    def backward(ctx, grad_output):
        return grad_output / ctx.num, None


avg_grad = AvgGrad.apply


# ---------------------------------------------------------------------------- #
# Losses
# ---------------------------------------------------------------------------- #
def smooth_cross_entropy(input, target, label_smoothing):
    """Cross entropy loss with label smoothing

    Args:
        input (torch.Tensor): [N, C]
        target (torch.Tensor): [N]
        label_smoothing (float): smoothing factor

    Returns:
        torch.Tensor: scalar
    """
    assert input.dim() == 2 and target.dim() == 1
    assert input.size(0) == target.size(0)
    batch_size, num_classes = input.shape
    one_hot = torch.zeros_like(input).scatter(1, target.unsqueeze(1), 1)
    smooth_one_hot = one_hot * (1 - label_smoothing) + torch.ones_like(input) * (label_smoothing / num_classes)
    log_prob = F.log_softmax(input, dim=1)
    loss = (-smooth_one_hot * log_prob).sum(1).mean()
    return loss


# ---------------------------------------------------------------------------- #
# Indexing
# ---------------------------------------------------------------------------- #


def batch_rot_with_axis(angle, rot_axis=2):
    assert angle.shape[-1] == 1
    rot_cos = torch.cos(angle)[..., 0]
    rot_sin = torch.sin(angle)[..., 0]
    j = (rot_axis + 1) % 3
    k = (rot_axis + 2) % 3
    rot_shape = list(angle.shape[:-1]) + [3, 3]
    rot_mat = torch.zeros(rot_shape, dtype=angle.dtype, device=angle.device)
    rot_mat[..., rot_axis, rot_axis] = 1
    rot_mat[..., j, j] = rot_mat[..., k, k] = rot_cos
    rot_mat[..., j, k] = -rot_sin
    rot_mat[..., k, j] = rot_sin
    return rot_mat
