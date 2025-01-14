import numpy as np
from numbers import Number
from pyrl.utils.data import is_seq_of, is_str, is_num, deepcopy

from pyrl.utils.meta import Registry, build_from_cfg


SCHEDULERS = Registry("scheduler of hyper-parameters")


class BaseScheduler:
    def __init__(self, init_values=None):
        self.niter = 0
        self.init_values = init_values

    def reset(self):
        self.niter = 0

    def get(self, *args, **kwargs):
        raise NotImplementedError

    def step(self, *args, **kwargs):
        self.niter += 1
        return self.get(*args, **kwargs)


@SCHEDULERS.register_module()
class FixedScheduler(BaseScheduler):
    def get(self, value=None, niter=None):
        return self.init_values if value is None else value

@SCHEDULERS.register_module()
class LmbdaScheduler(BaseScheduler):
    """
    Tune the hyper-parameter by the running steps
    """

    def __init__(self, lmbda, init_values=None):
        super(LmbdaScheduler, self).__init__(init_values)
        self.lmbda = lmbda

    def get(self, init_values=None, niter=None):
        niter = self.niter if niter is None else niter
        if self.init_values is None:
            self.init_values = init_values
        return self.lmbda(init_values, niter)


@SCHEDULERS.register_module()
class StepScheduler(BaseScheduler):
    def __init__(self, steps, gamma, init_values=None):
        super(StepScheduler, self).__init__(init_values)
        self.steps = np.sort(steps)
        self.gamma = gamma
        print(self.steps, gamma)

    def get(self, init_values=None, niter=None):
        niter = self.niter if niter is None else niter
        if self.init_values is None:
            self.init_values = init_values
        init_values = self.init_values
        step_index = np.searchsorted(self.steps, niter, side="right")
        gamma = self.gamma**step_index
        if is_num(init_values):
            return init_values * gamma
        elif isinstance(init_values, (tuple, list)):
            ret = []
            for x in init_values:
                ret.append(x * gamma)
            return type(init_values)(ret)
        else:
            ret = {}
            for key in init_values:
                ret[key] = init_values[key] * gamma
            return ret


@SCHEDULERS.register_module()
class KeyStepScheduler(BaseScheduler):
    def __init__(self, keys, steps, gammas, init_values=None):
        super(KeyStepScheduler, self).__init__(init_values)
        if is_str(keys):
            keys = [
                keys,
            ]
        if is_num(gammas):
            gammas = [
                gammas,
            ]
        if is_num(steps):
            steps = [
                [
                    steps,
                ]
            ]
        elif is_seq_of(steps, Number):
            steps = [
                steps,
            ]
        self.infos = {}
        for i, key in enumerate(keys):
            gamma = gammas[min(i, len(gammas) - 1)]
            step = steps[min(i, len(steps) - 1)]
            self.infos[key] = (deepcopy(step), gamma)
        print(self.infos)

    def get(self, init_values=None, niter=None):
        niter = self.niter if niter is None else niter
        ret_values = dict() if init_values is None else init_values
        if self.init_values is None:
            assert isinstance(init_values, dict)
            self.init_values = {key: init_values[key] for key in self.infos if key in init_values}
        init_values = self.init_values
        for key in self.infos:
            steps, gamma = self.infos[key]
            step_index = np.searchsorted(steps, niter, side="right")
            # print(init_values[key], gamma, step_index)
            ret_values[key] = init_values[key] * (gamma**step_index)
        return ret_values

def build_scheduler(cfg, default_args=None):
    return build_from_cfg(cfg, SCHEDULERS, default_args)
