import numpy as np
from collections import defaultdict
from typing import Union
from tqdm import tqdm
from itertools import count

from pyrl.utils.meta import get_logger, TqdmToLogger, parse_files
from pyrl.utils.data import DictArray, GDict, is_null, DataCoder, is_not_null
from pyrl.utils.file import get_total_size, FileCache
from .builder import REPLAYS, build_sampling


@REPLAYS.register_module()
class ReplayMemory:
    """
    This replay buffer is designed for RL, BRL.
    Replay buffer uses dict-array as basic data structure, which can be easily saved as hdf5 file.
    Also it utilize a asynchronized memory cache system to speed up the file loading process.
    See dict_array.py for more details.

    Two special keys for multiprocess and engineering usage
        is_truncated: to indicate if the trajectory is truncated here and the next sample from the same worker is from another trajectory.
        woker_index: to indicate which process generate this sample. Used in recurrent policy.
        is_valid: to indicate if this sample is useful. Used in recurrent policy.
    """

    def __init__(
        self,
        capacity,
        sampling_cfg=dict(type="OneStepTransition"),
        keys=None,
        keys_map=None,
        data_coder_cfg=None,
        buffer_filenames=None,
        cache_size=2048,
        num_samples=-1,
        num_procs=4,
        synchronized=True,  # For debug only which is slower than asynchronized file loading and data augmentation
        dynamic_loading=None,
        auto_buffer_resize=True,
        deterministic_loading=False,
    ):
        self.logger = get_logger()
        self.capacity = capacity
        self.memory = None
        self.position = 0
        self.running_count = 0
        self.dynamic_loading = dynamic_loading
        self.deterministic_loading = deterministic_loading
        self.num_samples = num_samples
        self.with_cache = None
        assert capacity > 0 or buffer_filenames is not None, "Capacity of the replay buffer must be positive or buffer_filenames must be provided."

        self.data_coder = None if is_null(data_coder_cfg) else DataCoder(**data_coder_cfg)
        buffer_filenames = self.parse_buffer_filenames(buffer_filenames)
        self.build_data_loader(
            buffer_filenames,
            cache_size,
            num_procs,
            synchronized,
            keys,
            keys_map,
            1 if (sampling_cfg is None or "horizon" not in sampling_cfg) else sampling_cfg["horizon"],
        )

        self.auto_buffer_resize = auto_buffer_resize
        self.reset()

        if sampling_cfg is not None:
            sampling_cfg["capacity"] = self.capacity
            sampling_cfg["no_random"] = sampling_cfg.get("no_random", False) or self.deterministic_loading
        self.sampling = build_sampling(sampling_cfg)
        if self.dynamic_loading:
            self.sampling.with_replacement = False

        self.load_files(buffer_filenames, keys, keys_map)

    def parse_buffer_filenames(self, buffer_filenames):
        if buffer_filenames is None:
            return None
        buffer_filenames = parse_files(buffer_filenames)
        if self.deterministic_loading:
            buffer_filenames = sorted(buffer_filenames)
            self.logger.warning("Sort files and change sampling strategy!")
        self.logger.info(f"Get {len(buffer_filenames)} files!")
        if len(buffer_filenames) == 0:
            self.logger.warning("Do not find any files!")
            buffer_filenames = None
        self.data_size = get_total_size(buffer_filenames, num_samples=self.num_samples)
        self.logger.info(f"Load {len(buffer_filenames)} files with {self.data_size} samples in total!")
        if self.capacity < 0:
            self.capacity = self.data_size
            self.logger.info(f"Set capacity to be data_size {self.capacity}!")
        if self.dynamic_loading is None:
            self.dynamic_loading = self.capacity < self.data_size
            if self.dynamic_loading:
                self.logger.info(f"Using dynamic loading!")
        return buffer_filenames

    def build_data_loader(self, buffer_filenames, cache_size, num_procs, synchronized, keys, keys_map, horizon):
        if buffer_filenames is None:
            self.file_loader = None
            self.dynamic_loading = False
            return

        # Cache utils does not support var input length recently
        self.with_cache = (self.data_coder is None) or (not self.data_coder.var_len_item)
        if self.dynamic_loading and cache_size != self.capacity:
            self.logger.warning("You should use same the cache_size as the capacity when dynamically loading files!")
            cache_size = self.capacity
        if not self.dynamic_loading and keys is not None:
            self.logger.warning("Some important keys may be dropped in buffer and the buffer cannot be extended!")
        if self.with_cache:
            self.file_loader = FileCache(
                buffer_filenames,
                min(cache_size, self.capacity),
                keys,
                self.data_coder,
                num_procs,
                synchronized=synchronized,
                num_samples=self.num_samples,
                horizon=horizon,
                keys_map=keys_map,
                deterministic_loading=self.deterministic_loading,
            )
            self.logger.info("Finish building file cache!")
        else:
            self.logger.info("Load without cache!")

    def load_files(self, buffer_filenames, keys, keys_map):
        if buffer_filenames is None:
            return

        if self.dynamic_loading:
            self.file_loader.run(auto_restart=False)
            items = self.file_loader.get()
            self.push_batch(items)
            self.file_loader.run(auto_restart=False)
        else:
            self.logger.info("Load all the data at one time!")
            if self.with_cache:
                tqdm_obj = tqdm(file=TqdmToLogger(), mininterval=10, total=self.data_size)
                while True:
                    self.file_loader.run(auto_restart=False)
                    items = self.file_loader.get()

                    if items is None:
                        break
                    self.push_batch(items)
                    tqdm_obj.update(len(items))
            else:
                # For Scannet recently which has vairant input size
                self.logger.info(f"Loading full dataset without cache system!")
                for filename in tqdm(file=TqdmToLogger(), mininterval=60)(buffer_filenames):
                    from h5py import File

                    file = File(filename, "r")
                    from pyrl.utils.file.cache_utils import META_KEYS

                    traj_keys = [key for key in list(file.keys()) if key not in META_KEYS]
                    traj_keys = sorted(traj_keys)
                    if self.num_samples > 0:
                        traj_keys = traj_keys[: self.num_samples]
                    data = DictArray.from_hdf5(filename, traj_keys)
                    if keys is not None:
                        data = data.select_by_keys(keys)
                    if is_not_null(self.data_coder):
                        data = self.data_coder.decode(data)
                    data = data.to_two_dims()
                    self.push_batch(data)
            self.logger.info(f"Finish file loading! Buffer length: {len(self)}, buffer size {self.memory.nbytes_all / 1024 / 1024} MB!")
            self.logger.info(f"Len of sampling buffer: {len(self.sampling)}")

    def __getitem__(self, key):
        return self.memory[key]

    def __setitem__(self, key, value):
        self.memory[key] = value

    def __getattr__(self, key):
        return getattr(self.memory, key, None)

    def __len__(self):
        return min(self.running_count, self.capacity)

    def reset(self):
        self.position = 0
        self.running_count = 0
        self.cached_traj = defaultdict(list)

        # self.memory = None
        if self.sampling is not None:
            self.sampling.reset()

    def push(self, item):
        if not isinstance(item, DictArray):
            item = GDict(item).slice(None, wrapper=False)  # Try to avoid copying
            item = DictArray(item)
        assert len(item) == 1, "Only support pushing one sample at one time!"
        self.push_batch(item)

    def push_list(self, items):
        for item in items:
            self.push(item)

    def push_batch(self, items: Union[DictArray, dict]):
        if not isinstance(items, DictArray):
            items = DictArray(items)
        if len(items) > self.capacity:
            items = items.slice(slice(0, self.capacity))

        if "worker_indices" not in items:
            items["worker_indices"] = np.zeros([len(items), 1], dtype=np.int32)
        if "is_truncated" not in items:
            items["is_truncated"] = np.zeros([len(items), 1], dtype=np.bool_)

        if self.memory is None:
            # Init the whole buffer
            self.memory = DictArray(items.slice(0), capacity=self.capacity)
        if self.position + len(items) > self.capacity:
            # Deal with buffer overflow
            final_size = self.capacity - self.position
            self.push_batch(items.slice(slice(0, final_size)))
            self.position = 0
            self.push_batch(items.slice(slice(final_size, len(items))))
        else:
            self.memory.assign(slice(self.position, self.position + len(items)), items)
            self.running_count += len(items)
            self.position = (self.position + len(items)) % self.capacity
            if self.sampling is not None:
                self.sampling.push_batch(items)

    def cache_trajectories(self, items, num=None):
        items = DictArray(items)
        pushed_count = 0
        for i in range(len(items)):
            item_i = items.slice(i, wrapper=False)
            # print(GDict(item_i).shape)
            worker_index = item_i["worker_indices"][0]
            cache = self.cached_traj[worker_index]
            cache.append(item_i)

            if item_i["episode_dones"][0]:
                if num is not None and len(cache) > num:
                    cache = cache[:num]
                    cache[-1]["is_truncated"] = np.ones(1, dtype=np.bool_)
                    num -= len(cache)
                self.push_list(cache)
                pushed_count += len(cache)
                del self.cached_traj[worker_index]
        return pushed_count

    def push_cached_trajectories(self, num=None):
        pushed_count = 0
        for worker_index in self.cached_traj:
            cache = self.cached_traj[worker_index]
            if num is not None:
                if num <= 0:
                    break
                if len(cache) > num:
                    cache = cache[:num]
                    num -= len(cache)
                    cache[-1]["is_truncated"] = np.ones(1, dtype=np.bool_)
            pushed_count += len(cache)
            self.push_list(cache)
        return pushed_count

    def update_all_items(self, items):
        self.memory.assign(slice(0, len(items)), items)

    def tail_mean(self, num):
        return self.tail(num).to_gdict().mean()

    def get_all(self):
        # Return all elements in replay buffer
        return self.memory.slice(slice(0, len(self)))

    def tail(self, num):
        assert num <= len(self), f"num={num} is larger than buffer length={len(self)}!"
        # print(num, self.position)
        if num <= self.position:
            # Return all elements in replay buffer
            return self.memory.slice(slice(self.position - num, self.position))
        else:
            part_1 = self.memory.slice(slice(self.capacity - num + self.position, self.capacity))
            part_2 = self.memory.slice(slice(0, self.position))
            return DictArray.concat([part_1, part_2], axis=0)

    def to_hdf5(self, file, traj_index=None):
        data = self.get_all()
        if traj_index is not None:
            # Save the whole replay buffer into one trajectory.
            # TODO: Parse the trajectories in replay buffer.
            data = GDict({f"traj_{traj_index}": data.memory})
        data.to_hdf5(file)

    def sample(self, batch_size, auto_restart=True, drop_last=True):
        if self.dynamic_loading and not drop_last:
            assert self.capacity % batch_size == 0

        batch_idx, is_valid = self.sampling.sample(batch_size, drop_last=drop_last, auto_restart=auto_restart and not self.dynamic_loading)
        # print(batch_idx)
        # exit(0)
        if batch_idx is None:
            # without replacement only
            if auto_restart or self.dynamic_loading:
                items = self.file_loader.get()
                if items is None:
                    return None
                assert self.position == 0, "cache size should equals to buffer size"
                self.sampling.reset()
                self.push_batch(items)
                self.file_loader.run(auto_restart=auto_restart)
                batch_idx, is_valid = self.sampling.sample(batch_size, drop_last=drop_last, auto_restart=auto_restart and not self.dynamic_loading)
            else:
                return None
        # print(batch_idx, is_valid, batch_idx.min(), batch_idx.max())
        # print(self.memory.shape)
        # exit(0)
        ret = self.memory.take(batch_idx)
        ret["is_valid"] = is_valid
        return ret

    def mini_batch_sampler(self, batch_size, drop_last=False, auto_restart=False, max_num_batches=-1):
        if self.sampling is not None:
            old_replacement = self.sampling.with_replacement
            self.sampling.with_replacement = False
            self.sampling.restart()
        for i in count(1):
            if i > max_num_batches and max_num_batches != -1:
                break
            items = self.sample(batch_size, auto_restart, drop_last)
            if items is None:
                self.sampling.with_replacement = old_replacement
                break
            yield items

    def close(self):
        if self.file_loader is not None:
            self.file_loader.close()

    def __del__(self):
        self.close()
