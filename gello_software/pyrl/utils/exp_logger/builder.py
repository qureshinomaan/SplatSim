from ..meta import Registry, build_from_cfg

EXP_LOGGER = Registry("logger")


def build_exp_logger(cfg):
    return build_from_cfg(cfg, EXP_LOGGER)
