import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Categorical, MixtureSameFamily
from pyrl.utils.torch import ScaledTanhNormal, ScaledNormal, CustomIndependent
from ..builder import REGRESSION
from .regression_base import ContinuousBaseHead


class GaussianBaseHead(ContinuousBaseHead):
    def __init__(
        self, bound=None, dim_output=None, nn_cfg=None, predict_std=True, init_log_std=-0.5, clip_return=False, num_heads=1, log_std_bound=[-20, 2]
    ):
        # bound is None means the action is unbounded.
        super(GaussianBaseHead, self).__init__(bound=bound, dim_output=dim_output, clip_return=clip_return, num_heads=num_heads, nn_cfg=nn_cfg)
        self.predict_std = predict_std
        self.log_std = None if predict_std else nn.Parameter(torch.ones(1, self.dim_output) * init_log_std)
        self.dim_feature = self.dim_output * (int(predict_std) + 1)
        if self.num_heads > 1:
            self.dim_feature = (self.dim_feature + 1) * self.num_heads
        self.log_std_min, self.log_std_max = log_std_bound

    def split_feature(self, feature, num_samples=1):
        assert feature.shape[-1] == self.dim_feature, f"{feature.shape, self.dim_feature}"

        if num_samples > 1:
            feature = feature.repeat_interleave(num_samples, dim=0)

        if self.num_heads > 1:
            logits = feature[..., : self.num_heads]
            feature = feature[..., self.num_heads :]
        else:
            logits = None

        if self.log_std is None:
            mean, log_std = feature.chunk(2, dim=-1)
        else:
            mean = feature
            log_std = self.log_std
        if self.num_heads > 1:
            pred_shape = list(mean.shape)
            pred_dim = pred_shape[-1] // self.num_heads
            mean_shape = pred_shape[:-1] + [self.num_heads, pred_dim]
            mean = mean.reshape(*mean_shape)
            if self.log_std is not None:
                log_std = self.log_std.expand_as(mean)
            else:
                log_std = log_std.reshape(*mean_shape)
        std = torch.clamp(log_std, min=self.log_std_min, max=self.log_std_max).exp()
        return logits, mean, std

    def return_with_mean_std(self, mean, std, dist_builder, mode, logits=None):
        if self.num_heads > 1:
            logits_max = logits.argmax(-1)
            logits_max = logits_max[..., None, None].repeat_interleave(mean.shape[-1], dim=-1)
        dist = dist_builder(mean, std)

        mean_ret = dist.mean
        std_ret = dist.stddev
        if self.num_heads > 1:
            mixture_distribution = Categorical(logits=logits)
            dist = MixtureSameFamily(mixture_distribution, dist)
            mean_ret = torch.gather(mean_ret, -2, logits_max).squeeze(-2)
            std_ret = torch.gather(std_ret, -2, logits_max).squeeze(-2)
        return self._get_results(dist, mean_ret, std_ret, mode)

    def extra_repr(self) -> str:
        return "predict_std={}, clip_return={}, num_heads={}".format(self.predict_std, self.clip_return, self.num_heads)


@REGRESSION.register_module()
class TanhGaussianHead(GaussianBaseHead):
    """
    ScaledTanhNomral, For SAC, CQL, Discor poliy network.
    The policy network will always output bounded value. Tanh(Gaussian(mean, std))
    """

    def __init__(self, *args, epsilon=1e-6, **kwargs):
        kwargs["clip_return"] = False
        super(TanhGaussianHead, self).__init__(*args, **kwargs)
        self.epsilon = epsilon

    def forward(self, feature, num_samples=1, mode="explore", **kwargs):
        feature = super(TanhGaussianHead, self).forward(feature)
        logits, mean, std = self.split_feature(feature, num_samples)
        dist_builder = lambda mean, std: CustomIndependent(ScaledTanhNormal(mean, std, self.scale, self.bias, self.epsilon), 1)
        return self.return_with_mean_std(mean, std, dist_builder, mode, logits)


@REGRESSION.register_module()
class GaussianHead(GaussianBaseHead):
    """
    Nomral.
    It will use a tanh head to output a bounded mean.
    """

    def __init__(self, *args, **kwargs):
        kwargs = dict(kwargs)
        if "clip_return" not in kwargs:
            kwargs["clip_return"] = True
        super(GaussianHead, self).__init__(*args, **kwargs)

    def forward(self, feature, num_samples=1, mode="explore", **kwargs):
        feature = super(GaussianHead, self).forward(feature)
        logits, mean, std = self.split_feature(feature, num_samples)
        if self.bound is not None:
            mean = torch.tanh(mean)
        dist_builder = lambda mean, std: CustomIndependent(ScaledNormal(mean, std, self.scale, self.bias), 1)
        return self.return_with_mean_std(mean, std, dist_builder, mode, logits)


@REGRESSION.register_module()
class SoftplusGaussianHead(GaussianBaseHead):
    """
    For PETS model network.
    """

    def __init__(self, *args, init_log_var_min=-1, init_log_var_max=0.5, clip_return=False, **kwargs):
        super(SoftplusGaussianHead, self).__init__(*args, clip_return=clip_return, **kwargs)
        self.log_var_min = nn.Parameter(torch.ones(1, self.dim_output).float() * init_log_var_min)
        self.log_var_max = nn.Parameter(torch.ones(1, self.dim_output).float() * init_log_var_max)

    def forward(self, feature, num_samples=1, mode="explore", **kwargs):
        feature = super(SoftplusGaussianHead, self).forward(feature)
        logits, mean, std = self.split_feature(feature, num_samples)
        log_var = std.log() * 2
        log_var = self.log_var_max - F.softplus(self.log_var_max - log_var)
        log_var = self.log_var_min + F.softplus(log_var - self.log_var_min)
        std = (log_var / 2).exp()
        dist_builder = lambda mean, std: CustomIndependent(ScaledNormal(mean, std, self.scale_prior, self.bias_prior), 1)
        return self.return_with_mean_std(mean, std, dist_builder, mode, logits)
