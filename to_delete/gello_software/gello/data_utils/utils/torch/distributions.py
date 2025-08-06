from torch.distributions import Normal, Independent, TransformedDistribution
from torch.distributions import Categorical
import torch
from math import log

class TransformedNormal(TransformedDistribution):
    def _transform(self, x):
        for transform in self.transforms:
            x = transform(x)
        return x

    @property
    def mean(self):
        return self._transform(self.base_dist.mean)

    @property
    def stddev(self):
        return None


class CustomCategorical(Categorical):
    def __init__(self, *args, **kwargs):
        super(CustomCategorical, self).__init__(*args, **kwargs)

    def log_prob(self, value):
        logits_dim = self.logits.ndim
        if value.ndim == logits_dim:
            assert value.shape[-1] == 1, f"Shape error {value.shape}"
            value = value[..., 0]
        return super(CustomCategorical, self).log_prob(value)


class ScaledNormal(Normal):
    def __init__(self, mean, std, scale_prior, bias_prior):
        # print(mean.shape, std.shape, scale_prior.shape, bias_prior.shape)
        super(ScaledNormal, self).__init__(mean * scale_prior + bias_prior, std * scale_prior)
        self.scale_prior, self.bias_prior = scale_prior, bias_prior

    def rsample_with_log_prob(self, sample_shape=torch.Size()):
        ret = self.rsample(sample_shape)
        log_p = self.log_prob(ret)
        return ret, log_p


class ScaledTanhNormal(Normal):
    def __init__(self, mean, std, scale_prior, bias_prior, epsilon=1e-6):
        super(ScaledTanhNormal, self).__init__(mean, std)
        self.scale_prior, self.bias_prior = scale_prior, bias_prior
        self.epsilon = epsilon
        self.log_2 = log(2)

    def log_prob_with_logit(self, x):
        log_prob = super(ScaledTanhNormal, self).log_prob(x)

        """
        tanh(x) = (exp(x) - exp(-x)) / (exp(x) + exp(-x))
          p(scale * tah(x))
        = p(x) / (scale * (1 - tanh(x) ** 2))
        
        log p = log p(x) - log(scale * (1 - tanh(x) ** 2))
        = log p(x) - log(scale) - log(1 - tanh(x) ** 2)
        
          log(1 - tanh(x) ** 2)
        = log(sech(x) ** 2)
        = 2 * log(sech(x)) 
        = 2 * log(2 / (exp(x) + exp(-x))
        = 2 * log(2 exp(-x) / (1 + exp(-2x))
        = 2 * log(2 exp(-x)) - log(1 + exp(-2x))
        = 2 * (log 2 - x - softplus(-2x))
        
        Torch implementation - Old
        log_prob -= torch.log(self.scale_prior * (1 - torch.tanh(x).pow(2)) + self.epsilon)
        
        New one uses softplus: 1 / beta * log(1 + exp(beta * x)) to avoid numerical issue
        # https://github.com/openai/spinningup/blob/master/spinup/algos/pytorch/sac/core.py
        
        log_prob -= torch.log(self.scale_prior) + 2 * (self.log_2 - x - F.softplus(-2 * x))
        """

        # tmp1 = torch.log(self.scale_prior * (1 - torch.tanh(x).pow(2)) + self.epsilon)
        # tmp2 = torch.log(self.scale_prior) + 2 * (self.log_2 - x - F.softplus(-2 * x))
        # torch.log(self.scale_prior) + 2 * (self.log_2 + x - F.softplus(2 * x))
        # print(tmp1, tmp2)
        # from IPython import embed

        # embed()

        # print("1", log_prob.mean())
        log_prob -= torch.log(self.scale_prior * (1 - torch.tanh(x).pow(2)) + self.epsilon)

        # log_prob -= torch.log(self.scale_prior) + 2 * (self.log_2 - x - F.softplus(-2 * x))
        # print("1.5", log_prob.mean())
        return log_prob

    def log_prob(self, x):
        x_ = self.un_transform(x)
        log_p = self.log_prob_with_logit(x_)
        return log_p

    def rsample(self, sample_shape=torch.Size()):
        return self.transform(super(ScaledTanhNormal, self).rsample(sample_shape))

    def sample(self, sample_shape=torch.Size()):
        return self.transform(super(ScaledTanhNormal, self).sample(sample_shape))

    @property
    def mean(self):
        return self.transform(super(ScaledTanhNormal, self).mean)

    def transform(self, x):
        return torch.tanh(x) * self.scale_prior + self.bias_prior

    def un_transform(self, x):
        return torch.atanh((x - self.bias_prior) / self.scale_prior)

    def rsample_with_log_prob(self, sample_shape=torch.Size()):
        logit = super(ScaledTanhNormal, self).rsample(sample_shape)
        log_prob = self.log_prob_with_logit(logit)
        return self.transform(logit), log_prob


class CustomIndependent(Independent):
    def rsample_with_log_prob(self, sample_shape=torch.Size()):
        sample, log_prob = self.base_dist.rsample_with_log_prob(sample_shape)
        from torch.distributions.utils import _sum_rightmost

        return sample, _sum_rightmost(log_prob, self.reinterpreted_batch_ndims)

    @property
    def stddev(self):
        if hasattr(self.base_dist, "stddev"):
            return self.base_dist.stddev
        else:
            return self.base_dist.variance.sqrt()

    @property
    def shape(self):
        for key in ["loc", "logits"]:
            if hasattr(self.base_dist, key):
                return getattr(self.base_dist, key).shape
        from IPython import embed

        embed()
        exit(0)


from copy import deepcopy
