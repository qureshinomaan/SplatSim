from pyrl.utils.torch.distributions import ScaledNormal
import torch.nn as nn, torch, numpy as np
from torch.nn import Parameter
from ..builder import REGRESSION
from pyrl.utils.data import is_num
from torch.distributions import Normal, Categorical, MixtureSameFamily
from pyrl.utils.torch import ExtendedModule, CustomIndependent
from .regression_base import ContinuousBaseHead


class DeterministicHead(ContinuousBaseHead):
    def __init__(self, bound=None, dim_output=None, clip_return=False, num_heads=1, nn_cfg=None, noise_std=0.1):
        super(DeterministicHead, self).__init__(bound=bound, dim_output=dim_output, clip_return=clip_return, num_heads=num_heads, nn_cfg=nn_cfg)
        self.dim_feature = self.dim_output if self.num_heads == 1 else (self.dim_output + 1) * self.num_heads
        if dim_output is not None:
            if is_num(noise_std):
                noise_std = np.ones(self.dim_output) * noise_std
            assert noise_std.shape[-1] == dim_output
        # The noise is the Gaussian noise on normalized action space for exploration.
        self.noise_std = Parameter(self.scale.data * torch.tensor(noise_std))

    def split_feature(self, feature, num_actions=1):
        assert feature.shape[-1] == self.dim_feature
        feature = feature.repeat_interleave(num_actions, dim=0)

        if self.num_heads > 1:
            logits = feature[..., : self.num_heads]
            feature = feature[..., self.num_heads :]
            pred_shape = list(feature.shape)
            pred_dim = pred_shape[-1] // self.num_heads
            mean_shape = pred_shape[:-1] + [self.num_heads, pred_dim]
            mean = feature.reshape(*mean_shape)
        else:
            logits = None
            mean = feature
        std = self.noise_std.data.expand_as(mean)
        return logits, mean, std

    def return_with_mean_std(self, mean, std, mode, logits=None):
        if self.num_heads > 1:
            logits_max = logits.argmax(-1)
            logits_max = logits_max[..., None, None].repeat_interleave(mean.shape[-1], dim=-1)

        if mode == "mean" or mode == "eval":
            ret = mean if logits is None else torch.gather(mean, -2, logits_max).squeeze(-2)
            return ret * self.scale + self.bias

        dist = CustomIndependent(ScaledNormal(mean, std, self.scale, self.bias), 1)
        mean_ret = dist.mean
        std_ret = dist.stddev
        if self.num_heads > 1:
            mixture_distribution = Categorical(logits=logits)
            dist = MixtureSameFamily(mixture_distribution, dist)
            mean_ret = torch.gather(mean_ret, -2, logits_max).squeeze(-2)
            std_ret = torch.gather(std_ret, -2, logits_max).squeeze(-2)
        return self._get_results(dist, mean_ret, std_ret, mode)


@REGRESSION.register_module()
class BasicHead(DeterministicHead):
    def forward(self, feature, num_actions=1, mode="explore", **kwargs):
        feature = super(BasicHead, self).forward(feature)
        logits, mean, std = self.split_feature(feature, num_actions)
        return self.return_with_mean_std(mean, std, mode, logits)


@REGRESSION.register_module()
class TanhHead(DeterministicHead):
    def forward(self, feature, num_actions=1, mode="explore", **kwargs):
        feature = super(TanhHead, self).forward(feature)
        logits, mean, std = self.split_feature(feature, num_actions)
        mean = torch.tanh(mean)
        return self.return_with_mean_std(mean, std, mode, logits)
