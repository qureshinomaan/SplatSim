import torch.nn as nn
from ..utils.meta import Registry, build_from_cfg
from copy import deepcopy


NETWORK = Registry("neural_network")
REGRESSION = Registry("regression")
APPLICATION = Registry("application")


def build_all(cfg, default_args=None):
    if cfg is None:
        return None

    if isinstance(cfg, (list, tuple)):
        return [build_all(cfg_, default_args) for cfg_ in cfg]
    else:
        for model_type in [NETWORK, REGRESSION, APPLICATION]:
            if cfg["type"] in model_type.module_dict:
                return build_from_cfg(cfg, model_type, default_args)

    raise RuntimeError(f"No this model type:{cfg['type']}!")


SHARED_KEYS = ["visual_nn", "rnn", "obs_feat", "prev_ac_feat", "recent_frame_feat"]


def build_target_network(network_cfg, network, shared_network=None, shared_backbone=False):
    from pyrl.utils.torch import disable_gradients, hard_update

    if shared_network is None:
        shared_network = network
    if shared_backbone:
        network_cfg = deepcopy(network_cfg)
        for name in SHARED_KEYS:
            item = getattr(shared_network.backbone, name, None)
            if item is not None:
                setattr(network_cfg.nn_cfg, f"{name}_cfg", None)
                setattr(network_cfg.nn_cfg, name, item)
        target_network = build_all(network_cfg)
    else:
        target_network = deepcopy(network)
    hard_update(target_network, network)
    disable_gradients(target_network, exclude=[id(_) for _ in network.parameters()])
    return target_network


def build_actor_critic(actor_cfg, critic_cfg, shared_backbone=False):
    if shared_backbone:
        assert (
            actor_cfg["nn_cfg"]["type"]
            in [
                "FrameMiners",
                "Visuomotor",
                "SequenceModel",
            ]
            or "Visuomotor" in actor_cfg["nn_cfg"]["type"]
        ), f"Only Visuomotor model could share backbone, actually visual backbone can be shared. Your mode has type {actor_cfg['nn_cfg']['type']}!"
        actor = build_all(actor_cfg)

        critic_cfg = deepcopy(critic_cfg)
        for name in SHARED_KEYS:
            item = getattr(actor.backbone, name, None)
            if item is not None:
                setattr(critic_cfg.nn_cfg, f"{name}_cfg", None)
                setattr(critic_cfg.nn_cfg, name, item)

        critic = build_all(critic_cfg)
        return actor, critic
    else:
        actor = build_all(actor_cfg)
        critic = build_all(critic_cfg)
        return actor, critic
