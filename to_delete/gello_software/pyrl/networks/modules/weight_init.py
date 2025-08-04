import numpy as np, torch.nn as nn
from pyrl.utils.meta import Registry


INIT = Registry("init")


@INIT.register_module()
def constant_init(module, val, bias=0):
    if hasattr(module, "weight") and module.weight is not None:
        nn.init.constant_(module.weight, val)
    elif hasattr(module, "kernel"):
        nn.init.constant_(module.kernel, val)
    if hasattr(module, "bias") and module.bias is not None:
        nn.init.constant_(module.bias, bias)


@INIT.register_module()
def xavier_init(module, gain=1, bias=0, distribution="normal"):
    assert distribution in ["uniform", "normal"]
    if distribution == "uniform":
        if hasattr(module, "weight"):
            nn.init.xavier_uniform_(module.weight, gain=gain)
    elif hasattr(module, "kernel"):
        nn.init.xavier_uniform_(module.kernel, gain=gain)
    else:
        if hasattr(module, "weight"):
            nn.init.xavier_normal_(module.weight, gain=gain)
        elif hasattr(module, "kernel"):
            nn.init.xavier_normal_(module.kernel, gain=gain)
    if hasattr(module, "bias") and module.bias is not None:
        nn.init.constant_(module.bias, bias)


@INIT.register_module()
def normal_init(module, mean=0, std=1, bias=0):
    if hasattr(module, "weight"):
        nn.init.normal_(module.weight, mean, std)
    elif hasattr(module, "kernel"):
        nn.init.normal_(module.kernel, mean, std)
    if hasattr(module, "bias") and module.bias is not None:
        nn.init.constant_(module.bias, bias)


@INIT.register_module()
def uniform_init(module, a=0, b=1, bias=0):
    if hasattr(module, "weight"):
        nn.init.uniform_(module.weight, a, b)
    elif hasattr(module, "kernel"):
        nn.init.uniform_(module.kernel, a, b)
    if hasattr(module, "bias") and module.bias is not None:
        nn.init.constant_(module.bias, bias)


@INIT.register_module()
def orthogonal_init(module, gain=1, bias=0):
    if hasattr(module, "weight"):
        nn.init.orthogonal_(module.weight, gain)
    elif hasattr(module, "kernel"):
        nn.init.orthogonal_(module.kernel, gain)
    if hasattr(module, "bias") and module.bias is not None:
        nn.init.constant_(module.bias, bias)


@INIT.register_module()
def delta_orthogonal_init(module, gain=1):
    # added delta-orthogonal init from https://arxiv.org/pdf/1806.05393.pdf
    if isinstance(module, nn.Linear):
        nn.init.orthogonal_(module.weight.data)
        if module.bias is not None:
            nn.init.constant_(module.bias, 0)
    elif isinstance(module, nn.Conv2d) or isinstance(module, nn.ConvTranspose2d):
        assert module.weight.size(2) == module.weight.size(3)
        nn.init.constant_(module.weight, 0)
        if module.bias is not None:
            nn.init.constant_(module.bias, 0)
        mid = module.weight.size(2) // 2
        nn.init.orthogonal_(module.weight.data[:, :, mid, mid], gain)


@INIT.register_module()
def kaiming_init(module, a=0, mode="fan_out", nonlinearity="relu", bias=0, distribution="normal"):
    assert distribution in ["uniform", "normal"]
    if distribution == "uniform":
        if hasattr(module, "weight"):
            nn.init.kaiming_uniform_(module.weight, a=a, mode=mode, nonlinearity=nonlinearity)
        elif hasattr(module, "kernel"):
            nn.init.kaiming_uniform_(module.kernel, a=a, mode=mode, nonlinearity=nonlinearity)
    else:
        if hasattr(module, "weight"):
            nn.init.kaiming_normal_(module.weight, a=a, mode=mode, nonlinearity=nonlinearity)
        elif hasattr(module, "kernel"):
            nn.init.kaiming_normal_(module.kernel, a=a, mode=mode, nonlinearity=nonlinearity)
    if hasattr(module, "bias") and module.bias is not None:
        nn.init.constant_(module.bias, bias)


@INIT.register_module()
def caffe2_xavier_init(module, bias=0):
    # `XavierFill` in Caffe2 corresponds to `kaiming_uniform_` in PyTorch
    # Acknowledgment to FAIR's internal code
    kaiming_init(module, a=1, mode="fan_in", nonlinearity="leaky_relu", distribution="uniform")


@INIT.register_module()
def bias_init_with_prob(prior_prob):
    """initialize conv/fc bias value according to giving probablity."""
    bias_init = float(-np.log((1 - prior_prob) / prior_prob))
    return bias_init


def build_init(cfg, *args, **kwargs):
    if cfg is None:
        return None
    if not isinstance(cfg, dict):
        raise TypeError("cfg must be a dict")
    if "type" not in cfg:
        raise KeyError('the cfg dict must contain the key "type"')
    cfg_ = cfg.copy()
    init_type = cfg_.pop("type")
    if init_type not in INIT:
        raise KeyError(f"Unrecognized init type {init_type}")
    else:
        init_func = INIT.get(init_type)
    kwargs.update(cfg_)
    init_func_ret = lambda _: init_func(_, *args, **kwargs)
    return init_func_ret
