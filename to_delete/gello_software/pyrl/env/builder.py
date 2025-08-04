from pyrl.utils.meta import Registry, build_from_cfg


ROLLOUTS = Registry("rollout")
EVALUATIONS = Registry("evaluation")

REPLAYS = Registry("replay")
SAMPLING = Registry("sampling")


def build_rollout(cfg, default_args=None):
    return build_from_cfg(cfg, ROLLOUTS, default_args)


def build_evaluation(cfg, default_args=None):
    cfg["type"] = "Evaluation"
    if cfg.get("num_procs", 1) > 1 and cfg.type == "Evaluation":
        cfg.type = "BatchEvaluation"
    elif cfg.get("type", "Evaluation") == "BatchEvaluation":
        print("Although we use only one thread, you still want to use BatchEvaluation!")
    return build_from_cfg(cfg, EVALUATIONS, default_args)


def build_replay(cfg, default_args=None):
    return build_from_cfg(cfg, REPLAYS, default_args)


def build_sampling(cfg, default_args=None):
    return build_from_cfg(cfg, SAMPLING, default_args)
