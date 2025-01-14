from gym.spaces import Box, Discrete, Space
from pyrl.utils.data import repeat
import numpy as np


class StackedDiscrete(Space):
    def __init__(self, num_envs, n):
        assert n >= 0
        self.n = n
        self.num_envs = num_envs
        super(StackedDiscrete, self).__init__((), np.int64)

    def sample(self):
        return self.np_random.randint(self.n, size=[self.num_envs, 1])

    def contains(self, x):
        return np.logical_and(0 <= x, x < self.n)

    def __repr__(self):
        return f"StackedDiscrete(n={self.n}, size=[{self.num_envs}, 1])"

    def __eq__(self, other):
        return isinstance(other, StackedDiscrete) and self.n == other.n and self.num_envs == other.num_envs


def stack_action_space(action_space, num):
    if isinstance(action_space, Box):
        low, high = action_space.low, action_space.high
        return Box(low=repeat(low[None], num, 0), high=repeat(high[None], num, 0))  # By default is float32
    elif isinstance(action_space, Discrete):
        return StackedDiscrete(num, action_space.n)
    else:
        print("Unknown action space:", action_space, type(action_space))
        raise NotImplementedError()


def unstack_action_space(action_space):
    if isinstance(action_space, Box):
        low, high = action_space.low[0], action_space.high[0]
        return Box(low=low, high=high)
    elif isinstance(action_space, StackedDiscrete):
        return Discrete(action_space.n)
    else:
        raise NotImplementedError()
