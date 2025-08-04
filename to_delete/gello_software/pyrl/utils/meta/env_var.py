import os
from .network_utils import find_free_port


def add_env_var():
    default_values = {
        "NUMEXPR_MAX_THREADS": "1",
        "MKL_NUM_THREADS": "1",
        "OMP_NUM_THREADS": "1",
        "CUDA_DEVICE_ORDER": "PCI_BUS_ID",
        "DISPLAY": "0",
        "MUJOCO_GL": "glfw",
        "D4RL_SUPPRESS_IMPORT_ERROR": "1",
    }
    for key, value in default_values.items():
        os.environ[key] = os.environ.get(key, value)


def add_dist_var(rank, world_size):
    os.environ["PYRL_RANK"] = f"{rank}"
    os.environ["PYRL_WORLD_SIZE"] = f"{world_size}"
    os.environ["MASTER_ADDR"] = "localhost"

    os.environ["MASTER_PORT"] = str(find_free_port(12355))
    os.environ["PYRL_TCP_PORT"] = str(find_free_port(15015))


def get_world_rank():
    if "PYRL_RANK" not in os.environ:
        return 0
    return eval(os.environ["PYRL_RANK"])


def get_world_size():
    if "PYRL_WORLD_SIZE" not in os.environ:
        return 1
    return eval(os.environ["PYRL_WORLD_SIZE"])


def get_dist_info():
    return get_world_rank(), get_world_size()


def is_debug_mode():
    if "PYRL_DEBUG" not in os.environ:
        return 0
    return eval(os.environ["PYRL_DEBUG"])
