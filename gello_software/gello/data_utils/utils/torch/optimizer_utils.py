import numpy as np
import copy, inspect, torch
from pyrl.utils.meta import Registry, build_from_cfg
from pyrl.utils.data import regex_match


OPTIMIZERS = Registry("optimizer")

def register_torch_optimizers():
    torch_optimizers = []
    for module_name in dir(torch.optim):
        if module_name.startswith("__"):
            continue
        _optim = getattr(torch.optim, module_name)
        if inspect.isclass(_optim) and issubclass(_optim, torch.optim.Optimizer):
            OPTIMIZERS.register_module()(_optim)
            torch_optimizers.append(module_name)
    return torch_optimizers


TORCH_OPTIMIZERS = register_torch_optimizers()


def get_mean_lr(optimizer, mean=True):
    ret = []
    for param_group in optimizer.param_groups:
        ret.append(param_group["lr"])
    return np.mean(ret) if mean else ret


def build_optimizer(model, cfg):
    cfg = copy.deepcopy(cfg)
    constructor_type = cfg.pop("constructor", "default") # keyword "type" is saved for specify optimizer.
    if constructor_type == "default":
        param_cfg = cfg.pop("param_cfg", None)

        param_i_template = copy.deepcopy(cfg)
        param_i_template.pop("type", None)

        params = []
        existing_params = []
        if hasattr(model, "named_parameters"):
            # It is a mode not a Parameter.
            for name, param in model.named_parameters():
                if id(param) in existing_params or not param.requires_grad:
                    continue
                param_i = copy.deepcopy(param_i_template)
                existing_params.append(id(param))
                if param_cfg is not None:
                    for pattern, param_config in param_cfg.items():
                        if regex_match(name, pattern):
                            param_i = param_config
                            break
                if param_i is None:
                    continue
                param_i = {'params': param}
                params.append(param_i)
        else:
            params = [model, ]
        cfg["params"] = params
        optimizer = build_from_cfg(cfg, OPTIMIZERS)
    else:
        raise NotImplementedError
    return optimizer
