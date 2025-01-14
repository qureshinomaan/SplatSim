from functools import wraps
import numpy as np
import torch
from pyrl.utils.math import split_num
from pyrl.utils.meta import get_logger
from pyrl.utils.data import DictArray, to_np, GDict, to_torch

from contextlib import ContextDecorator


class ProfilerBlockCuda(ContextDecorator):
    def __init__(self, *args, **kwargs) -> None:
        from sapien.core import ProfilerBlock

        self.profiler_block = ProfilerBlock(*args, **kwargs)

    def __enter__(self):
        torch.cuda.synchronize()
        self.profiler_block.__enter__()

    def __exit__(self, arg0, arg1, arg2):
        torch.cuda.synchronize()
        self.profiler_block.__exit__(arg0, arg1, arg2)


def disable_gradients(network, exclude=[]):
    for param in network.parameters():
        if id(param) in exclude:
            continue
        param.requires_grad = False


def worker_init_fn(worker_id):
    """The function is designed for pytorch multi-process dataloader.
    Note that we use the pytorch random generator to generate a base_seed. Please try to be consistent.
    References:
        https://pytorch.org/docs/stable/notes/faq.html#dataloader-workers-random-seed
    """
    base_seed = torch.IntTensor(1).random_().item()
    np.random.seed(base_seed + worker_id)


def get_seq_info(done_mask):
    """
    It will sort the length of the sequence to improve the performance
    input: done_mask [L]
    return: index [#seq, max_seq_len]; sorted_idx [#seq]; is_valid [#seq, max_seq_len]
    """
    done_mask = to_np(done_mask)

    indices = np.where(done_mask)[0]
    indices = np.insert(indices, 0, -1, axis=0)
    len_seq = indices[1:] - indices[:-1]

    sorted_idx = np.argsort(-len_seq, kind="stable")  # From max to min
    max_len = len_seq[sorted_idx[0]]
    index = np.zeros([len(sorted_idx), max_len], dtype=np.int64)
    is_valid = np.zeros([len(sorted_idx), max_len], dtype=np.bool_)

    for i, idx in enumerate(sorted_idx):
        index[i, : len_seq[idx]] = indices[idx] + np.arange(1, len_seq[idx] + 1)
        is_valid[i, : len_seq[idx]] = True
    return index, sorted_idx, is_valid


def run_with_mini_batch(
    function,
    *args,
    batch_size=None,
    wrapper=True,
    device=None,
    ret_device=None,
    episode_dones=None,
    is_recurrent=False,
    recurrent_horizon=-1,
    rnn_mode="base",
    **kwargs,
):
    """
    Run a pytorch function with mini-batch when the batch size of dat is very large.
    :param function: the function
    :param data: the input data which should be in dict array structure
    :param batch_size: the num of samples in the whole batch, it is num of episodes * episode length for recurrent data.
    :return: all the outputs.
    """
    capacity = None
    assert rnn_mode in ["base", "full_states", "with_states"], f"{rnn_mode} is not supported by rnn_mode"
    # print('In mini batch', batch_size, DictArray(kwargs).shape)

    def process_kwargs(x):
        if x is None or len(x) == 0:
            return None
        nonlocal capacity, device, ret_device
        x = DictArray(x)
        capacity, data_device = x.capacity, x.one_device
        device = data_device if device is None else device
        ret_device = data_device if device is None else ret_device
        return x

    args = process_kwargs(list(args))
    kwargs = process_kwargs(dict(kwargs))
    assert capacity is not None, "Inputs do not have capacity!"

    if batch_size is None:
        batch_size = capacity
    recurrent_run = (episode_dones is not None) and is_recurrent
    rnn_states, next_rnn_states, latest_rnn_states = None, None, None

    if not recurrent_run:
        # assert rnn_mode != "full_states", "We cannot provide full states in this mode!"
        ret = []
        for i in range(0, capacity, batch_size):
            num_i = min(capacity - i, batch_size)
            args_i = args.slice(slice(i, i + num_i)).to_torch(device=device, wrapper=False) if args is not None else []
            kwargs_i = kwargs.slice(slice(i, i + num_i)).to_torch(device=device, wrapper=False) if kwargs is not None else {}
            if rnn_mode != "base":
                kwargs_i["rnn_mode"] = rnn_mode
            ret.append(GDict(function(*args_i, **kwargs_i)).to_torch(device=ret_device, wrapper=False))
        ret = DictArray.concat(ret, axis=0, wrapper=wrapper)
        if rnn_mode != "base":
            ret, latest_rnn_states = ret
    else:
        assert episode_dones is not None and rnn_mode in ["base", "full_states"], f"Flags {episode_dones is not None}, {rnn_mode}"

        if kwargs is not None:
            assert kwargs.memory.pop("rnn_states", None) is None, "You do not need to provide rnn_states!"
        index, sorted_index, is_valid = get_seq_info(episode_dones)
        # print(index, sorted_index, is_valid)

        def build_empty_list(num):
            return [None for i in range(num)]

        capacity = len(index)
        ret, rnn_states, next_rnn_states = build_empty_list(capacity), build_empty_list(capacity), build_empty_list(capacity)
        batch_size = batch_size // recurrent_horizon
        for i in range(0, capacity, batch_size):
            # Batch over trajectories and then Batch over horizon
            num_i = min(capacity - i, batch_size)
            index_i, is_valid_i = index[i : i + num_i], to_torch(is_valid[i : i + num_i], device=device)
            max_len = is_valid_i[0].sum().item()
            args_i = args.slice(index_i) if args is not None else None
            kwargs_i = kwargs.slice(index_i) if kwargs is not None else None
            latest_rnn_states, rnn_states_i, next_rnn_states_i, ret_i = None, [], [], []
            for j in range(0, max_len, recurrent_horizon):
                slice_ij = slice(j, j + min(max_len - j, recurrent_horizon))
                args_ij = args_i.slice(slice_ij, axis=1, wrapper=False) if args_i is not None else []
                kwargs_ij = kwargs_i.slice(slice_ij, axis=1, wrapper=False) if kwargs_i is not None else {}
                is_valid_ij = is_valid_i[:, slice_ij]

                args_ij, kwargs_ij = GDict([args_ij, kwargs_ij]).to_torch(device=device)
                ret_ij, [rnn_states_ij, next_rnn_states_ij, latest_rnn_states] = function(
                    *args_ij, **kwargs_ij, rnn_states=latest_rnn_states, is_valid=is_valid_ij, rnn_mode="full_states"
                )

                def process_reults(results):
                    return GDict(results).copy().to_torch(device=ret_device, wrapper=False)

                rnn_states_i.append(process_reults(rnn_states_ij))
                next_rnn_states_i.append(process_reults(next_rnn_states_ij))
                ret_i.append(process_reults(ret_ij))

            ret_i = DictArray.concat(ret_i, axis=1)
            rnn_states_i = DictArray.concat(rnn_states_i, axis=1)
            next_rnn_states_i = DictArray.concat(next_rnn_states_i, axis=1)
            for j, ori_idx in enumerate(sorted_index[i : i + num_i]):
                ret[ori_idx] = ret_i.slice(j).slice(is_valid_i[j], wrapper=False)
                rnn_states[ori_idx] = rnn_states_i.slice(j).slice(is_valid_i[j], wrapper=False)
                next_rnn_states[ori_idx] = next_rnn_states_i.slice(j).slice(is_valid_i[j], wrapper=False)
        ret = DictArray.concat(ret, axis=0, wrapper=wrapper)
        rnn_states = DictArray.concat(rnn_states, axis=0, wrapper=wrapper)
        next_rnn_states = DictArray.concat(next_rnn_states, axis=0, wrapper=wrapper)
    if rnn_mode == "base":
        return ret
    elif rnn_mode == "with_states":
        return ret, latest_rnn_states
    else:
        return ret, [rnn_states, next_rnn_states, None]


def mini_batch(wrapper_=True):
    def actual_mini_batch(f):
        wraps(f)

        def wrapper(*args, batch_size=None, wrapper=None, device=None, ret_device=None, **kwargs):
            if wrapper is None:
                wrapper = wrapper_
            # print(batch_size, dict(kwargs))

            return run_with_mini_batch(f, *args, **kwargs, batch_size=batch_size, wrapper=wrapper, device=device, ret_device=ret_device)

        return wrapper

    return actual_mini_batch
