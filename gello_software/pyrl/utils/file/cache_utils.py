import numpy as np, time
from random import shuffle
from io import BytesIO
from h5py import File, Group
from tqdm import tqdm

from pyrl.utils.meta import get_filename_suffix, Worker, get_logger
from pyrl.utils.math import split_num
from pyrl.utils.data import GDict, DictArray, SharedDictArray, is_h5, is_not_null, is_null, is_str
from .serialization import load, dump
from .record_utils import load_record_indices, load_items_from_record


def is_h5_traj(h5):
    if isinstance(h5, str):
        if get_filename_suffix(h5) != "h5":
            return False
        with File(h5, "r") as f:
            return is_h5_traj(f)
    if not is_h5(h5):
        return False, None
    keys = list(h5.keys())
    for key in keys:
        if key.startswith("traj") or key.startswith("dict_str_traj"):
            return True, key
    return False, None


def get_filetypes(filenames):
    ret_types = []
    for name in filenames:
        suffix = get_filename_suffix(name)
        if suffix in ["record"]:
            ret_types.append("one-step")
        elif suffix in ["record_episode"]:
            ret_types.append("episode")
        elif suffix == "h5":
            if is_h5_traj(name)[0]:
                ret_types.append("episode")
            else:
                ret_types.append("one-step")
        else:
            ret_types.append(None)
    return ret_types


def compatible_with_horizon(filetypes, horizon):
    if horizon != 1:
        for filetype in filetypes:
            if filetype != "episode":
                logger = get_logger()
                logger.error(f"Random shuffled buffer do not support {horizon}-step sampling")
                exit(0)


def decode_items(items, outputs, start_index=0, data_coder=None):
    if is_null(data_coder):
        return items
    for i in range(len(items)):
        item = items.slice(i)
        item = data_coder.decode(item)
        outputs.assign(start_index + i, item)


def decode_worker(input_buf_name, len_input, out_buf_infos, start_index=None, data_coder=None, woker_id=None):
    start_time = time.time()
    from multiprocessing.shared_memory import SharedMemory

    input_buf = SharedMemory(name=input_buf_name)
    input_bytes = input_buf.buf[:len_input].tobytes()

    input_buf.close()
    input_items = load(BytesIO(input_bytes), file_format="pkl")
    input_items = DictArray(input_items)

    outputs = SharedDictArray(None, *out_buf_infos)
    decode_items(input_items, outputs, start_index, data_coder)
    return time.time() - start_time


META_KEYS = ["meta"]


def len_items(items):
    if isinstance(items, GDict):
        items = items.memory
    if hasattr(items, "shape"):
        return items.shape[0]
    ret = None
    if isinstance(items, (dict, Group)):
        for key in items:
            # print(type(items), key)
            ret = len_items(items[key])
            if ret is not None:
                break
    else:
        assert isinstance(items, (list, tuple))
        for item in items:
            ret = len_items(item)
            if ret is not None:
                break
    return ret


def purify_items(items, keys=None, full=True, one_episode=False, keys_map=None):
    if keys is None:
        return items
    if isinstance(items, dict):
        items = purify_items(GDict(items), keys, full, one_episode, keys_map).memory
    elif isinstance(items, GDict):
        if keys is not None:
            items = items.select_by_keys(keys)
        if keys_map is not None:
            for key, mapped_key in keys_map.items():
                if key in items:
                    item = items[key]
                    del items[key]
                    items[mapped_key] = item
        items.to_two_dims()
        if "worker_indices" not in items and full:
            # print(len_items(items), GDict(items).shape)
            items["worker_indices"] = np.zeros((len_items(items), 1), dtype=np.int32)
        if "dones" in items and full:
            items["dones"] = items["dones"].astype(np.bool_)
            if "episode_dones" not in items:
                items["episode_dones"] = items["dones"]
        if "episode_dones" in items:
            items["episode_dones"] = items["episode_dones"].astype(np.bool_)
            if one_episode:
                items["episode_dones"][-1] = True
    return items


def create_shared_dict_array_from_files(filenames, capacity, data_coder, keys, keys_map=None):
    # print(filenames, capacity, data_coder, keys)
    # exit(0)
    filename = filenames[0]
    file_suffix = get_filename_suffix(filename)
    if file_suffix == "h5":
        sign, h5_idx = is_h5_traj(filename)
        if sign:
            item = GDict.from_hdf5(filename, keys=h5_idx)
            item = DictArray(item)
        else:
            item = DictArray.from_hdf5(filename)
    else:
        item = load_items_from_record(filename, None, 1, None, True)
        if file_suffix == "record":
            item = DictArray(item, capacity=1)
        elif file_suffix == "record_episode":
            item = DictArray(item)
    item = purify_items(item, keys, keys_map=keys_map)
    item = item.slice(0)
    if is_not_null(data_coder):
        item = data_coder.decode(item)
    item = DictArray(item, capacity)
    return SharedDictArray(item)


def get_total_size(filenames, record_indices=None, num_samples=-1):
    if is_str(filenames):
        assert record_indices is None
        filenames = [filenames]
    if record_indices is None:
        record_indices = load_record_indices(filenames)
    else:
        assert len(record_indices) == len(filenames)
    ret = 0
    from pyrl.utils.meta import TqdmToLogger

    filenames = tqdm(filenames, file=TqdmToLogger(), mininterval=10)
    for i, filename in enumerate(filenames):
        st = time.time()
        file_suffix = get_filename_suffix(filename)
        record_indices_i = record_indices[i]
        if file_suffix == "h5":  # h5
            size = 0
            file = File(filename, "r")
            if is_h5_traj(file)[0]:
                keys = [key for key in list(file.keys()) if key not in META_KEYS]
                if num_samples != -1:
                    keys = keys[:num_samples]
                for key in keys:
                    size += len_items(file[key])
            else:
                # Normal h5 dataset
                len_f = len_items(file)
                size += min(len_f, num_samples) if num_samples != -1 else len_f
            file.close()
        else:
            if file_suffix == "record":
                size = len(record_indices_i)
                if num_samples != -1:
                    size = min(num_samples, size)
            elif file_suffix == "record_episode":
                num = len(record_indices_i)
                if num_samples != -1:
                    num = min(num, num_samples)
                # TODO: Check why this is fater than following one
                file_obj = open(filename, "rb")
                size = 0
                for i in range(num):
                    item = load_items_from_record(file_obj, record_indices_i, i, None, True)
                    size += len(item["actions"])
                file_obj.close()
            else:
                raise NotImplementedError
        ret += size
    # exit(0)
    return ret


class FileCacheWorker:
    """
    Use a seperate process to loading elements in files.
    All the elements in the files should be able to be represented by a DictArray
    """

    def __init__(
        self,
        filenames,
        capacity,
        keys,
        keys_map,
        buffer_infos,
        data_coder=None,
        num_procs=4,
        num_samples=-1,
        horizon=1,
        deterministic_loading=False,
        **kwargs,
    ):
        # keys_map is done after purifying the keys.
        self.data_coder = data_coder
        self.num_procs = num_procs
        base_seed = np.random.randint(0, int(1e9))
        if data_coder is not None and self.num_procs > 1:
            self.workers = [Worker(decode_worker, i, base_seed=base_seed + i) for i in range(num_procs)]
        else:
            self.num_procs = 1

        self.filenames = filenames
        self.filesuffix = [get_filename_suffix(_) for _ in self.filenames]
        self.filetypes = get_filetypes(self.filenames)
        self.record_indices = load_record_indices(self.filenames)
        self.len_files = self._compute_len_files()
        compatible_with_horizon(self.filetypes, horizon)

        self.keys = keys
        self.keys_map = keys_map
        self.capacity = capacity
        self.num_files = len(self.filenames)

        self.current_file = None
        self.file_index = 0
        self.traj_index = 0  # For h5
        self.item_index = 0  # For h5 and record
        self.current_keys = None  # For h5
        self.deterministic_loading = deterministic_loading
        self.num_samples = int(1e20) if num_samples == -1 else num_samples  # h5, record-episode: num of trajs, record: num of samples:

        self.cache_buffer = SharedDictArray(None, *buffer_infos)
        logger = get_logger()

        if self.num_procs > 1:
            self.input_buffers = [None for i in range(num_procs)]
            self.output_buffers = [SharedDictArray(self.cache_buffer.to_dict_array().slice(slice(0, 1))) for i in range(num_procs)]

        logger.info(
            f"Length of cache: {capacity}, cache size {self.cache_buffer.nbytes_all / 1024 / 1024} MB, cache shape {self.cache_buffer.shape}!"
        )
        self.reset()

    def reset(self):
        # Shuffle the files and begin to push files to buffer.
        index = list(range(len(self.filenames)))
        if not self.deterministic_loading:
            shuffle(index)
            self.filenames = [self.filenames[i] for i in index]
            self.filesuffix = [self.filesuffix[i] for i in index]
            self.filetypes = [self.filetypes[i] for i in index]
            self.record_indices = [self.record_indices[i] for i in index]
            self.len_files = [self.len_files[i] for i in index]

        if self.current_file is not None:
            self.current_file.close()
            self.current_file = None
        self.file_index = 0
        self.traj_index = 0
        self.item_index = 0
        self.current_keys = None

    def _compute_len_files(self):
        ret = []
        for i, record_index in enumerate(self.record_indices):
            if record_index is None:  # h5
                size = 0
                file = File(self.filenames[i], "r")
                if is_h5_traj(file)[0]:
                    # RL trajectory
                    size = len([key for key in file.keys() if key not in META_KEYS])
                else:
                    size = len_items(file)
                file.close()
            else:
                # record
                size = len(record_index)
            ret.append(size)
        return ret

    @property
    def size(self):
        return get_total_size(self.filenames, self.record_indices, self.num_samples)

    def get_next_file(self, auto_restart=False):
        filetype = self.filetypes[self.file_index]
        max_index = min(self.len_files[self.file_index], self.num_samples)
        # print(self.file_index, filetype, self.item_index, self.traj_index, max_index)
        if (filetype == "episode" and self.traj_index < max_index) or (filetype == "one-step" and self.item_index < max_index):
            pass
        elif self.file_index < len(self.filenames) - 1:
            if self.current_file is not None:
                self.current_file.close()
                self.current_file = None
            self.file_index += 1
            self.traj_index = 0
            self.item_index = 0
            self.current_keys = None
            self.cached_items = None
            self.len_cached_items = 0
        elif auto_restart:
            self.reset()
        else:
            return None
        if self.current_file is None:
            filename = self.filenames[self.file_index]
            file_suffix = get_filename_suffix(filename)
            if file_suffix == "h5":
                self.current_file = File(filename, "r")
                if is_h5_traj(self.current_file)[0]:
                    self.current_keys = sorted([key for key in self.current_file.keys() if key not in META_KEYS])
                else:
                    self.cached_items = DictArray.from_hdf5(self.current_file)
                    self.len_cached_items = len_items(self.cached_items)
            else:
                self.current_file = open(filename, "rb")
        return self.file_index

    def get_next_items(self, max_num, auto_restart=False):
        file_index = self.get_next_file(auto_restart)
        if file_index is None:
            return None
        filename = self.filenames[file_index]
        record_indices = self.record_indices[self.file_index]
        filesuffix = self.filesuffix[file_index]

        if filesuffix == "record":
            num = min(min(len(record_indices), self.num_samples) - self.item_index, max_num)
            items = load_items_from_record(self.current_file, record_indices, self.item_index, self.item_index + num, True)
            items = DictArray.stack(items, wrapper=True)
            items = purify_items(items, keys=self.keys, one_episode=False, keys_map=self.keys_map)
            self.item_index += num
        elif filesuffix == "h5" and not is_h5_traj(self.current_file)[0]:
            # Normal dataset
            num = min(min(self.len_cached_items, self.num_samples) - self.item_index, max_num)
            items = self.cached_items.slice(slice(self.item_index, self.item_index + num))
            items = purify_items(items, keys=self.keys, one_episode=False, keys_map=self.keys_map)
            self.item_index += num
        else:
            if filesuffix == "h5":
                # Trajectory dataset
                key = self.current_keys[self.traj_index]
                items = DictArray.from_hdf5(self.current_file[key])
            elif filesuffix == "record_episode":
                items = load_items_from_record(self.current_file, record_indices, self.traj_index, None, True)
                items = DictArray(items)
            else:
                raise NotImplementedError
            num_samples = len(items)
            num = min(max_num, num_samples - self.item_index)
            items = items.slice(slice(self.item_index, self.item_index + num))
            if num + self.item_index == num_samples:
                self.item_index = 0
                self.traj_index += 1
            else:
                self.item_index += num
            items = purify_items(items, keys=self.keys, one_episode=True, keys_map=self.keys_map)
        return items

    def fetch_next_buffer(self, auto_restart=False):
        num_items = 0
        ret = []
        while num_items < self.capacity:
            items = self.get_next_items(self.capacity - num_items, auto_restart)
            if items is None or len(items) == 0:
                break
            num_items += len(items)
            ret.append(items)
        if len(ret) == 0:
            return num_items
        ret = DictArray.concat(ret, axis=0)
        if self.data_coder is not None:
            if self.num_procs == 1:
                decode_items(ret, self.cache_buffer, 0, self.data_coder)
            else:
                num_procs, splt_items = split_num(num_items, self.num_procs)
                st = time.time()
                start_index = 0

                for i in range(num_procs):
                    item_i = ret.slice(slice(start_index, start_index + splt_items[i]), wrapper=False)
                    buffer_content = dump(item_i, file_format="pkl")

                    if self.input_buffers[i] is None or self.input_buffers[i].size < len(buffer_content):
                        if self.input_buffers[i] is None:
                            new_size = len(buffer_content)
                        else:
                            new_size = max(self.input_buffers[i].size * 2, len(buffer_content))
                            self.input_buffers[i].close()
                            self.input_buffers[i].unlink()
                        from multiprocessing.shared_memory import SharedMemory

                        self.input_buffers[i] = SharedMemory(size=new_size, create=True)
                    buffer_i = self.input_buffers[i]

                    buffer_i.buf[: len(buffer_content)] = buffer_content
                    self.workers[i].ask(
                        input_buf_name=buffer_i.name,
                        len_input=len(buffer_content),
                        out_buf_infos=self.cache_buffer.get_infos(),
                        start_index=start_index,
                        data_coder=self.data_coder,
                    )
                    start_index += splt_items[i]

                for i in range(num_procs):
                    process_time = self.workers[i].get()
        else:
            ret = ret.to_two_dims()
            self.cache_buffer.assign(range(num_items), ret)
        return num_items

    def close(self):
        if self.num_procs <= 1:
            return
        for i in range(self.num_procs):
            self.workers[i].close()
            if self.input_buffers[i] is not None:
                self.input_buffers[i].close()
                self.input_buffers[i].unlink()


class FileCache:
    def __init__(self, filenames, capacity, keys, data_coder, num_procs=4, synchronized=False, keys_map=None, **kwargs):
        self.capacity = capacity
        self.shared_buffer = create_shared_dict_array_from_files(filenames, capacity, data_coder, keys, keys_map=keys_map)
        buffer_infos = self.shared_buffer.get_infos()
        self.synchronized = synchronized
        self.num_valid_items = 0
        if synchronized:
            self.worker = FileCacheWorker(filenames, capacity, keys, keys_map, buffer_infos, data_coder, num_procs, **kwargs)
        else:
            seed = np.random.randint(int(1e9))
            self.worker = Worker(
                FileCacheWorker,
                None,
                seed,
                False,
                filenames=filenames,
                capacity=capacity,
                keys=keys,
                keys_map=keys_map,
                buffer_infos=buffer_infos,
                data_coder=data_coder,
                num_procs=num_procs,
                **kwargs,
            )

    def run(self, auto_restart=False):
        if self.synchronized:
            self.num_valid_items = self.worker.fetch_next_buffer(auto_restart=auto_restart)
        else:
            self.worker.call("fetch_next_buffer", auto_restart=auto_restart)

    def get(self):
        if not self.synchronized:
            # Wait for buffer is fully filled.
            self.num_valid_items = self.worker.wait()
        ret = self.shared_buffer.to_dict_array()
        if self.num_valid_items < self.capacity:
            ret = ret.slice(slice(0, self.num_valid_items))
        if self.num_valid_items > 0:
            return ret
        else:
            return None

    def close(self):
        if hasattr(self, "synchronized") and not self.synchronized:
            if isinstance(self.worker, Worker):
                self.worker.call("close")
                self.worker.close()
            else:
                self.worker.close()
