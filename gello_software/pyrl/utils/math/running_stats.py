from collections import deque
import numpy as np
from pyrl.utils.data import is_num


class MovingAverage:
    def __init__(self, size=10):
        self.size = size
        self.memory = deque(maxlen=size)

    def add(self, x):
        if is_num(x):
            x = [x]
        for i in x:
            if i not in [np.inf, np.nan, -np.inf]:
                self.memory.append(i)

    def mean(self):
        return np.mean(self.memory) if len(self.memory) > 0 else 0

    def std(self):
        return np.std(self.memory) if len(self.memory) > 0 else 0


class RunningMeanStd:
    """
    Calculates the running mean and std of a data stream:
        https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Parallel_algorithm
    """

    def __init__(self, mean=0, std=0):
        self.mean, self.var = mean, std**2
        self.n = 0

    @property
    def std(self):
        return np.maximum(np.sqrt(self.var), 1e-8)

    def add(self, x):
        """
        Parallel algorithm: a is the history, b is the new one.
        m = n * var
        """
        new_mean, new_var = np.mean(x, axis=0), np.var(x, axis=0)
        new_n = len(x)

        n = self.n + new_n
        delta = new_mean - self.mean
        m2_a = self.n * self.var
        m2_b = new_n * new_var
        m2 = m2_a + m2_b + delta**2 * (self.n * new_n) / n
        self.mean = self.mean + delta * new_n / n
        self.var = m2 / n
        self.n = n
