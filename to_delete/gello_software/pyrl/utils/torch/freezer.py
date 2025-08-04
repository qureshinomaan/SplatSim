"""
From Jiayuan

Helpers for operating modules/parameters
"""

import re
import torch.nn as nn
from ..data.string_utils import any_string, regex_match


def get_frozen_params(module):
    return [name for name, params in module.named_parameters() if not params.requires_grad]


def get_frozen_modules(module):
    return [name for name, m in module.named_modules() if not m.training]


def print_frozen_modules_and_params(module, logger=None):
    _print = print if logger is None else logger.info
    for name in get_frozen_modules(module):
        _print("Module {} is frozen.".format(name))
    for name in get_frozen_params(module):
        _print("Params {} is frozen.".format(name))


def apply_params(module, patterns=any_string, requires_grad=False):
    """Apply freeze/unfreeze on parameters

    Args:
        module (torch.nn.Module): the module to apply
        patterns (sequence of str): strings which define all the patterns of interests
        requires_grad (bool, optional): whether to freeze params

    """
    if isinstance(patterns, str):
        patterns = [patterns]
    for name, params in module.named_parameters():
        for pattern in patterns:
            assert isinstance(pattern, str)
            if regex_match(name, pattern):
                params.requires_grad = requires_grad


def apply_modules(module, patterns=any_string, mode=False, prefix=""):
    """Apply train/eval on modules

    Args:
        module (torch.nn.Module): the module to apply
        patterns (sequence of str): strings which define all the patterns of interests
        mode (bool, optional): whether to set the module training mode
        prefix (str, optional)

    """
    if isinstance(patterns, str):
        patterns = [patterns]
    for name, m in module._modules.items():
        for pattern in patterns:
            assert isinstance(pattern, str)
            full_name = prefix + ("." if prefix else "") + name
            if regex_match(full_name, pattern):
                # avoid redundant call
                print(name)
                m.train(mode)
            else:
                apply_modules(m, patterns, mode=mode, prefix=full_name)


def freeze_modules(module, patterns=any_string):
    """Freeze modules by matching patterns"""
    apply_modules(module, patterns, mode=False)


def unfreeze_modules(module, patterns=any_string):
    """Unfreeze module by matching patterns"""
    apply_modules(module, patterns, mode=True)


def freeze_params(module, patterns=any_string):
    """Freeze parameters by matching patterns"""
    apply_params(module, patterns, requires_grad=False)


def unfreeze_params(module, patterns=any_string):
    """Unfreeze parameters by matching patterns"""
    apply_params(module, patterns, requires_grad=True)


def apply_bn(module, mode, requires_grad):
    """Modify batch normalization in the module

    Args:
        module (nn.Module): the module to operate
        mode (bool): train/eval mode
        requires_grad (bool): whether parameters require gradients

    Notes:
        Note that the difference between the behaviors of BatchNorm.eval() and BatchNorm(track_running_stats=False)

    """
    for m in module.modules():
        if isinstance(m, (nn.BatchNorm1d, nn.BatchNorm2d, nn.BatchNorm3d)):
            m.train(mode)
            for params in m.parameters():
                params.requires_grad = requires_grad


def freeze_bn(module):
    apply_bn(module, mode=False, requires_grad=False)
