import os.path as osp, subprocess, sys, time
from collections import defaultdict
from pathlib import Path
from importlib import import_module


def get_PIL_version():
    try:
        import PIL
    except ImportError:
        return "None"
    else:
        return f"{PIL.__version__}"


def collect_base_env():
    """Collect information from system environments.
    Returns:
        dict: The environment information. The following fields are contained.
            - sys.platform: The variable of ``sys.platform``.
            - python: python version.
            - CUDA available: Bool, indicating if CUDA is available.
            - GPU devices: Device type of each GPU.
            - CUDA_HOME (optional): The env var ``CUDA_HOME``.
            - nvcc (optional): nvcc version.
            - gcc: gcc version, "n/a" if gcc is not installed.
            - pytorch: pytorch version.
            - pytorch compiling details: The output of ``torch.__config__.show()``.
            - torchvision (optional): torchvision version.
            - opencv: opencv version.
            - PIL: PIL version.
    """
    env_info = {}
    env_info["sys.platform"] = sys.platform
    env_info["python"] = sys.version.replace("\n", "")

    import torch

    cuda_available = torch.cuda.is_available()
    env_info["CUDA available"] = cuda_available

    if cuda_available:
        devices = defaultdict(list)
        for k in range(torch.cuda.device_count()):
            devices[torch.cuda.get_device_name(k)].append(str(k))
        for name, device_ids in devices.items():
            env_info["GPU " + ",".join(device_ids)] = name

        from torch.utils.cpp_extension import CUDA_HOME

        env_info["CUDA_HOME"] = CUDA_HOME

        if CUDA_HOME is not None and osp.isdir(CUDA_HOME):
            try:
                nvcc = osp.join(CUDA_HOME, "bin/nvcc")
                nvcc = subprocess.check_output(f'"{nvcc}" -V | tail -n1', shell=True)
                nvcc = nvcc.decode("utf-8").strip()
            except subprocess.SubprocessError:
                nvcc = "Not Available"
            env_info["nvcc"] = nvcc

        env_info["Num of GPUs"] = torch.cuda.device_count()
    else:
        env_info["Num of GPUs"] = 0
    try:
        gcc = subprocess.check_output("gcc --version | head -n1", shell=True)
        gcc = gcc.decode("utf-8").strip()
        env_info["gcc"] = gcc
    except subprocess.CalledProcessError:  # gcc is unavailable
        env_info["gcc"] = "n/a"

    env_info["pytorch"] = torch.__version__
    env_info["pytorch compiling details"] = torch.__config__.show()

    try:
        import torchvision

        env_info["torchvision"] = torchvision.__version__
    except ModuleNotFoundError:
        pass

    try:
        import cv2
        env_info["opencv"] = cv2.__version__
    except ModuleNotFoundError:
        pass

    try:
        import pyrl

        env_info["pyrl"] = pyrl.__version__
    except ModuleNotFoundError:
        pass

    return env_info


def collect_env():
    """Collect information from system environments."""
    env_info = collect_base_env()
    return env_info


def get_package_meta(package_name):
    try:
        package = import_module(package_name)
    except:
        return ""
    from pyrl.utils.external.git_utils import get_git_repo_info

    git_info = get_git_repo_info(package)
    try:
        version = package.__version__
    except:
        version = None

    ret = []
    if version is not None:
        ret.append(f"version: {version}")
    if git_info is not None:
        ret.append(f"git commit: {git_info}")
    return ", ".join(ret)


def get_meta_info():
    ret = {"meta_collect_time": time.strftime("%Y-%m-%d-%H:%M:%S", time.gmtime(time.time() - 7 * 3600))}  # CA time
    for print_name, package_name in [
        ["pyrl", "pyrl"],
        ["ManiSkill", "mani_skill"],
        ["ManiSkill-Callback", "maniskill"],
        ["ManiSkill2", "mani_skill2"],
    ]:
        info_i = get_package_meta(package_name)
        if len(info_i) > 0:
            ret[print_name] = info_i
    return ret


def check_reproducibility():
    from pyrl.utils.external.git_utils import get_git_modified

    flag = True
    for name in ["pyrl"]:
        untracked, unstashed = get_git_modified(name)
        if len(untracked) > 0 or len(unstashed) > 0:
            print(f"Untracked files in repo {name}")
            for name in untracked:
                print(name)
            print(f"Untracked files in repo {name}")
            for name in unstashed:
                print(name)
            flag = False

    if not flag:
        exit(0)


def log_meta_info(logger, meta_info=None):
    if meta_info is None:
        meta_info = get_meta_info()
    for key in meta_info:
        logger.info(f"{key}: {meta_info[key]}")


if __name__ == "__main__":
    for name, val in collect_env().items():
        print(f"{name}: {val}")
