"""
Modified from https://github.com/open-mmlab/mmcv/blob/master/mmcv/runner/dist_utils.py
"""
import functools
import os
import subprocess
from collections import OrderedDict

import torch
import torch.multiprocessing as mp
from torch import distributed as dist
from torch._utils import _flatten_dense_tensors, _take_tensors, _unflatten_dense_tensors
from pyrl.utils.meta import get_dist_info


def init_dist(launcher, backend="nccl", **kwargs):
    if mp.get_start_method(allow_none=True) is None:
        mp.set_start_method("spawn")
    if launcher == "pytorch":
        _init_dist_pytorch(backend, **kwargs)
    elif launcher == "mpi":
        _init_dist_mpi(backend, **kwargs)
    elif launcher == "slurm":
        _init_dist_slurm(backend, **kwargs)
    else:
        raise ValueError(f"Invalid launcher type: {launcher}")


def cleanup_dist():
    dist.destroy_process_group()


def _init_dist_pytorch(backend, **kwargs):
    # TODO: use local_rank instead of rank % num_gpus
    rank = int(os.environ["PYRL_RANK"])
    num_gpus = torch.cuda.device_count()
    torch.cuda.set_device(rank % num_gpus)
    dist.init_process_group(backend=backend, **kwargs)


def _init_dist_mpi(backend, **kwargs):
    # TODO: use local_rank instead of rank % num_gpus
    rank = int(os.environ["OMPI_COMM_WORLD_RANK"])
    num_gpus = torch.cuda.device_count()
    torch.cuda.set_device(rank % num_gpus)
    dist.init_process_group(backend=backend, **kwargs)


def _init_dist_slurm(backend, port=None):
    """Initialize slurm distributed training environment.
    If argument ``port`` is not specified, then the master port will be system environment variable ``MASTER_PORT``.
    If ``MASTER_PORT`` is not in system environment variable, then a default port ``29500`` will be used.
    Args:
        backend (str): Backend of torch.distributed.
        port (int, optional): Master port. Defaults to None.
    """
    proc_id = int(os.environ["SLURM_PROCID"])
    ntasks = int(os.environ["SLURM_NTASKS"])
    node_list = os.environ["SLURM_NODELIST"]
    num_gpus = torch.cuda.device_count()
    torch.cuda.set_device(proc_id % num_gpus)
    addr = subprocess.getoutput(f"scontrol show hostname {node_list} | head -n1")
    # specify master port
    if port is not None:
        os.environ["MASTER_PORT"] = str(port)
    elif "MASTER_PORT" in os.environ:
        pass  # use MASTER_PORT in the environment variable
    else:
        # 29500 is torch.distributed default port
        os.environ["MASTER_PORT"] = "29500"
    # use MASTER_ADDR in the environment variable if it already exists
    if "MASTER_ADDR" not in os.environ:
        os.environ["MASTER_ADDR"] = addr
    os.environ["WORLD_SIZE"] = str(ntasks)
    os.environ["LOCAL_RANK"] = str(proc_id % num_gpus)
    os.environ["RANK"] = str(proc_id)
    dist.init_process_group(backend=backend)


def master_only(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        world_rank, _ = get_dist_info()
        if world_rank == 0:
            return func(*args, **kwargs)

    return wrapper


def allreduce_params(params, coalesce=True, bucket_size_mb=-1):
    """Allreduce parameters.
    Args:
        params (list[torch.Parameters]): List of parameters or buffers of a model.
        coalesce (bool, optional): Whether allreduce parameters as a whole. Defaults to True.
        bucket_size_mb (int, optional): Size of bucket, the unit is MB. Defaults to -1.
    """
    _, world_size = get_dist_info()
    if world_size == 1:
        return
    params = [param.data for param in params]
    if coalesce:
        _allreduce_coalesced(params, world_size, bucket_size_mb)
    else:
        for tensor in params:
            dist.all_reduce(tensor.div_(world_size))


def allreduce_grads(params, coalesce=True, bucket_size_mb=-1):
    """Allreduce gradients.
    Args:
        params (list[torch.Parameters]): List of parameters of a model
        coalesce (bool, optional): Whether allreduce parameters as a whole. Defaults to True.
        bucket_size_mb (int, optional): Size of bucket, the unit is MB. Defaults to -1.
    """
    grads = [param.grad.data for param in params if param.requires_grad and param.grad is not None]
    _, world_size = get_dist_info()
    if world_size == 1:
        return
    if coalesce:
        _allreduce_coalesced(grads, world_size, bucket_size_mb)
    else:
        for tensor in grads:
            dist.all_reduce(tensor.div_(world_size))


def _allreduce_coalesced(tensors, world_size, bucket_size_mb=-1):
    if bucket_size_mb > 0:
        bucket_size_bytes = bucket_size_mb * 1024 * 1024
        buckets = _take_tensors(tensors, bucket_size_bytes)
    else:
        buckets = OrderedDict()
        for tensor in tensors:
            tp = tensor.type()
            if tp not in buckets:
                buckets[tp] = []
            buckets[tp].append(tensor)
        buckets = buckets.values()

    for bucket in buckets:
        flat_tensors = _flatten_dense_tensors(bucket)
        dist.all_reduce(flat_tensors)
        flat_tensors.div_(world_size)
        for tensor, synced in zip(bucket, _unflatten_dense_tensors(flat_tensors, bucket)):
            tensor.copy_(synced)


def barrier():
    # Syncrhonize all process
    _, world_size = get_dist_info()
    if world_size > 1:
        dist.barrier()


tcp_store = None


class DistVar:
    def __init__(self, name, dtype, is_dist=True):
        self.name = name
        self.is_dist = is_dist
        self.dtype = dtype
        self.value = 0 if dtype == "int" else ""
        if self.is_dist:
            if self.dtype == "int":
                self.add(0)
            else:
                self.set("")

    def set(self, value):
        if self.is_dist:
            get_tcp_store().set(self.name, value)
        else:
            self.value = value

    def add(self, value):
        if self.is_dist:
            get_tcp_store().add(self.name, value)
        else:
            self.value += value

    def get(self):
        if self.is_dist:
            ret = get_tcp_store().get(self.name)
            if self.dtype == "int":
                ret = eval(ret)
            return ret
        else:
            return self.value

    def __del__(self):
        try:
            if self.is_dist:
                get_tcp_store().delete_key(self.name)
        except:
            exit(-1)


def get_tcp_store():
    global tcp_store
    if tcp_store is None:
        from datetime import timedelta

        world_rank, world_size = get_dist_info()
        tcp_port = int(os.environ["PYRL_TCP_PORT"])  # 15015
        if world_rank == 0:
            tcp_store = dist.TCPStore("127.0.0.1", tcp_port, world_size, True, timedelta(seconds=30))
        else:
            tcp_store = dist.TCPStore("127.0.0.1", tcp_port, world_size, False)
    return tcp_store


def build_dist_var(name, dtype="str"):
    _, world_size = get_dist_info()
    assert dtype in ["int", "str"]
    return DistVar(name, dtype, is_dist=world_size > 1)
