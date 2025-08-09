import torch
import numpy as np
import torch.nn as nn
from .module_utils import ExtendedModule


class RunningMeanStdTorch(ExtendedModule):
    """
    Calculates the running mean and std of a data stream:
        https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Parallel_algorithm
    """

    def __init__(self, shape, mean=0, var=1, clip_max=None, with_std=True):
        super(RunningMeanStdTorch, self).__init__()
        self.with_std = with_std
        self._mean = nn.Parameter(torch.tensor(np.ones(shape) * mean, dtype=torch.float64), requires_grad=False)
        if with_std:
            self._var = nn.Parameter(torch.tensor(np.ones(shape) * var, dtype=torch.float64), requires_grad=False)
        self.n = nn.Parameter(torch.tensor(0, dtype=torch.int64), requires_grad=False)
        self.clip_max = clip_max

    @property
    @torch.no_grad()
    def std(self):
        assert self.with_std
        return torch.clamp(torch.sqrt(self._var.data), min=1e-8).float()

    @property
    @torch.no_grad()
    def mean(self):
        return self._mean.data.float()

    @torch.no_grad()
    def normalize(self, x):
        if self.with_std:
            x = (x - self.mean) / self.std
            if self.clip_max is not None:
                x = torch.clamp(x, -self.clip_max, self.clip_max)
        else:
            # Clip value to [-clip_max * mean, clip_max * mean]
            x = torch.clamp(x, -self.clip_max * self.mean, self.clip_max * self.mean)
        return x

    @torch.no_grad()
    def add(self, x, noramlize=True):
        """
        Parallel algorithm: a is the history, b is the new one.
        m = n * var
        """
        new_n = x.shape[0]
        n = self.n.data + new_n
        new_mean = torch.mean(x, dim=0)
        delta = new_mean - self._mean
        self._mean.data = self._mean.data + delta * new_n / n

        if self.with_std:
            if new_n == 1:
                new_var = new_mean * 0
            else:
                new_var = torch.var(x, dim=0)
            m2_a = self.n * self._var
            m2_b = new_n * new_var
            m2 = m2_a + m2_b + delta**2 * new_n * (self.n / n)
            self._var.data = m2 / n

        self.n.data = n
        return self.normalize(x) if noramlize else x

    def sync(self):
        # A hacked version. I just average means, vars, n over all processes.
        from pyrl.utils.data import GDict
        from .distributed_utils import barrier

        barrier()
        double_n = self.n.data.double()
        GDict([self._mean.data, self._var.data, double_n]).allreduce(device=self.device)
        self.n.data = torch.round(double_n).long()


class RunningSecondMomentumTorch(RunningMeanStdTorch):
    def __init__(self, shape, clip_max=5):
        super(RunningSecondMomentumTorch, self).__init__(shape=shape, clip_max=clip_max, with_std=False)

    @property
    @torch.no_grad()
    def mean(self):
        return self._mean.data.sqrt().float()

    @torch.no_grad()
    def add(self, x):
        super(RunningSecondMomentumTorch, self).add(x**2, noramlize=False)
        return self.normalize(x)


class MovingMeanStdTorch(nn.Module):
    """
    Calculates the running mean and std of a data stream:
        https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Parallel_algorithm
    """

    def __init__(self, shape, mean=0, var=1, momentum=0.5):
        super(MovingMeanStdTorch, self).__init__()
        self._mean = nn.Parameter(torch.tensor(np.ones(shape) * mean, dtype=torch.float64), requires_grad=False)
        self._var = nn.Parameter(torch.tensor(np.ones(shape) * var, dtype=torch.float64), requires_grad=False)
        self.n = nn.Parameter(torch.tensor(0, dtype=torch.int64), requires_grad=False)
        self.momentum = momentum

    @property
    @torch.no_grad()
    def std(self):
        return torch.clamp(torch.sqrt(self._var.data), min=1e-8).float()

    @property
    @torch.no_grad()
    def mean(self):
        return self._mean.data.float()

    @torch.no_grad()
    def normalize(self, x):
        return (x - self.mean) / self.std

    @torch.no_grad()
    def add(self, x):
        """
        Parallel algorithm: a is the history, b is the new one.
        m = n * var
        """
        self.n += 1
        self._mean.data = self._mean.data * self.momentum + x.mean(dim=0) * (1 - self.momentum)
        self._var.data = self._var.data * self.momentum + x.var(dim=0) * (1 - self.momentum)
        return (x - self.mean) / self.std
