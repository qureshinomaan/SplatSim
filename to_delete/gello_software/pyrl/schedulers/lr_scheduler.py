from pyrl.utils.meta import Registry, build_from_cfg
import torch.optim.lr_scheduler as lr_scheduler

LRSCHEDULERS = Registry("scheduler of pytorch learning rate")


for scheduler in [
    lr_scheduler.LambdaLR,
    lr_scheduler.MultiplicativeLR,
    lr_scheduler.StepLR,
    lr_scheduler.MultiStepLR,
    lr_scheduler.ConstantLR,
    lr_scheduler.LinearLR,
    lr_scheduler.ExponentialLR,
    lr_scheduler.CosineAnnealingLR,
    lr_scheduler.ChainedScheduler,
    lr_scheduler.SequentialLR,
    lr_scheduler.ReduceLROnPlateau,
    lr_scheduler.CyclicLR,
    lr_scheduler.OneCycleLR,
    lr_scheduler.CosineAnnealingWarmRestarts,
]:
    LRSCHEDULERS.register_module(module=scheduler)


def build_lr_scheduler(cfg, default_args=None):
    if cfg.get("type", None) == "LambdaLR":
        assert cfg.get("lr_lambda") is not None
        if isinstance(cfg["lr_lambda"], str):
            cfg["lr_lambda"] = eval(cfg["lr_lambda"])

    return build_from_cfg(cfg, LRSCHEDULERS, default_args)
