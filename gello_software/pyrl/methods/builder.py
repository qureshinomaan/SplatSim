from ..utils.meta import Registry, build_from_cfg


MFRL = Registry("mfrl")  # Model free RL


def build_agent(cfg, default_args=None):
    for agent_type in [MFRL]:
        if cfg["type"] in agent_type:
            return build_from_cfg(cfg, agent_type, default_args)
    return None
