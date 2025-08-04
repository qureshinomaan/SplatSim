try:
    from .dm_control_utils import register_dmc_envs

    register_dmc_envs()
except:
    pass

try:
    from .adroit_utils import register_adroit_envs

    register_adroit_envs()
except:
    pass


try:
    from .meta_world_utils import register_meta_world_envs

    register_meta_world_envs()
except:
    pass

from .simple_dist_env import DistEnv
