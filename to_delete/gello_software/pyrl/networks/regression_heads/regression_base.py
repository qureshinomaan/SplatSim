import torch
from torch.nn import Parameter
import numpy as np

from pyrl.utils.data import is_num, is_not_null, to_np
from pyrl.utils.torch import ExtendedModule, CustomCategorical
from ..builder import build_all, REGRESSION


class ContinuousBaseHead(ExtendedModule):
    def __init__(self, bound=None, dim_output=None, nn_cfg=None, clip_return=False, num_heads=1):
        super(ContinuousBaseHead, self).__init__()
        self.bound = bound
        self.net = build_all(nn_cfg)
        self.clip_return = clip_return and is_not_null(bound)

        if is_not_null(bound):
            if is_num(bound[0]):
                bound[0] = np.ones(dim_output) * bound[0]
            if is_num(bound[1]):
                bound[1] = np.ones(dim_output) * bound[1]
            assert (to_np(bound[0].shape) == to_np(bound[1].shape)).all()
            assert dim_output is None or bound[0].shape[-1] == dim_output
            dim_output = bound[0].shape[-1]
            if bound[0].ndim > 1:
                assert bound[0].ndim == 2 and bound[0].shape[0] == num_heads and num_heads > 1
            self.lb, self.ub = [Parameter(torch.tensor(bound[i]), requires_grad=False) for i in [0, 1]]
            self.log_uniform_prob = torch.log(1.0 / ((self.ub - self.lb).data)).sum().item()
            self.scale = Parameter(torch.tensor(bound[1] - bound[0]) / 2, requires_grad=False)
            self.bias = Parameter(torch.tensor(bound[0] + bound[1]) / 2, requires_grad=False)
        else:
            self.scale, self.bias = 1, 0

        self.dim_output = dim_output
        self.num_heads = num_heads
        self.dim_feature = None

    def uniform(self, sample_shape):
        r = torch.rand(sample_shape, self.dim_output, device=self.device)
        return (r * self.ub + (1 - r) * self.lb), torch.ones(sample_shape, device=self.device) * self.log_uniform_prob

    def clamp(self, x):
        if self.clip_return:
            x = torch.clamp(x, min=self.lb, max=self.ub)
        return x

    def forward(self, feature, **kwargs):
        return self.net(feature) if self.net is not None else feature

    def _get_results(self, dist, mean, std, mode):
        if mode == "max-entropy":
            mode_parts = ["rsample-with-neg-logp"]
        else:
            mode_parts = mode.split("_")

        ret = []
        for mode_i in mode_parts:
            if mode_i in ["mean", "eval"]:
                ret_i = self.clamp(mean)
            elif mode_i in ["explore", "sample"]:
                ret_i = self.clamp(dist.rsample() if dist.has_rsample else dist.sample())
            elif mode_i in ["std"]:
                ret_i = std
            elif mode_i in ["log_std"]:
                ret_i = std.log()
            elif mode_i == "dist":
                ret_i = dist
            elif mode_i in ["entropy"]:
                ret_i = dist.entropy()
            elif mode_i in ["rsample-with-neg-logp"]:
                [sample, log_p] = dist.rsample_with_log_prob()
                ret_i = [sample, -log_p[..., None]]
            ret.append(ret_i)
        return ret[0] if len(ret) == 1 else ret


@REGRESSION.register_module()
class DiscreteBaseHead(ExtendedModule):
    def __init__(self, num_choices, num_heads=1, **kwargs):
        super(DiscreteBaseHead, self).__init__()
        assert num_heads == 1, "We only support one head recently."
        self.num_choices = num_choices
        self.num_heads = num_heads

    def forward(self, feature, num_actions=1, mode="explore", **kwargs):
        assert feature.shape[-1] == self.num_choices * self.num_heads
        feature = feature.repeat_interleave(num_actions, dim=0)
        dist = CustomCategorical(logits=feature)
        greedy_action = feature.argmax(dim=-1, keepdim=True)
        if mode == "max-entropy":
            mode_parts = ["p", "entropy"]
        else:
            mode_parts = mode.split("_")

        ret = []

        for mode_i in mode_parts:
            if mode_i in ["mean", "eval", "greedy"]:
                ret_i = greedy_action
            elif mode_i in ["explore", "sample"]:
                ret_i = dist.rsample() if dist.has_rsample else dist.sample()
                ret_i = ret_i[..., None]  # [B, 1]
            elif mode_i == "dist":
                ret_i = dist
            elif mode_i in ["entropy"]:
                ret_i = dist.entropy()[..., None]  # [B,..., 1]
            elif mode_i in ["neg-logp"]:  # Log of p the greedy action
                ret_i = -dist.log_prob(greedy_action)
            elif mode_i in ["feature", "logits"]:
                ret_i = feature
            elif mode_i in ["prob", "p"]:
                ret_i = dist.probs
            ret.append(ret_i)
        return ret[0] if len(ret) == 1 else ret

    def extra_repr(self) -> str:
        return f"num_actions={self.num_choices}, num_head={self.num_heads}"
