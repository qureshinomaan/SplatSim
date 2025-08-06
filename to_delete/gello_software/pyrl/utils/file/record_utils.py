import os.path as osp
from shutil import rmtree
import struct
import time
from typing import Union
from io import BytesIO
import numpy as np
from pathlib import Path

from .hash_utils import masked_crc
from .serialization import dump, load
from ..data import DictArray, GDict, shuffle, select_by_index
from ..meta import check_files_exist, mkdir_or_exist, get_filename_suffix, replace_suffix, get_time_stamp, symlink, get_logger
from ..math import split_num


"""
TF-Record-Style Dataset
   1. We call one file a record, which should has suffix '.record' or '.record_episode'
"""


def write_item_to_record(item, data_file, index_file=None, serialized=False):
    """
    Write a python object to the record.
    """
    if not serialized:
        item = dump(item, file_format="pkl")
    length = len(item)
    length_bytes = struct.pack("<Q", length)
    current = data_file.tell()
    data_file.write(length_bytes)
    data_file.write(masked_crc(length_bytes))
    data_file.write(item)
    data_file.write(masked_crc(item))
    if index_file is not None:
        index_file.write(f"{current} {data_file.tell()}\n")


length_bytes, crc_bytes, data_buffer = None, None, None


def read_record(data_file, start_offset=None, end_offset=None, serialized=False):
    global length_bytes, crc_bytes, data_buffer
    if length_bytes is None:
        length_bytes = bytearray(8)
        crc_bytes = bytearray(4)
        data_buffer = bytearray(128 * 1024 * 1024)  # 128M
    if start_offset is not None:
        data_file.seek(start_offset)
    if end_offset is None:
        end_offset = osp.getsize(data_file.name)
    ret = []

    while data_file.tell() < end_offset:
        if data_file.readinto(length_bytes) != 8:
            raise RuntimeError("Failed to read the record size.")
        if data_file.readinto(crc_bytes) != 4:
            raise RuntimeError("Failed to read the start token.")
        (length,) = struct.unpack("<Q", length_bytes)
        if length > len(data_buffer):
            data_buffer = data_buffer.zfill(int(length * 1.5))
        data_buffer_content = memoryview(data_buffer)[:length]
        if data_file.readinto(data_buffer_content) != length:
            raise RuntimeError("Failed to read the record.")
        if data_file.readinto(crc_bytes) != 4:
            raise RuntimeError("Failed to read the end token.")
        item = bytes(data_buffer_content)
        if serialized:
            item = load(BytesIO(item), file_format="pkl")
        ret.append(item)
    return ret


def load_items_from_record(data_file, indices=None, start_idx=None, end_idx=None, serialized=False):
    if indices is None:
        assert isinstance(data_file, str)
        indices = get_index_filenames(data_file)
    if isinstance(data_file, str):
        open_file = True
        data_file = open(data_file, "rb")
    else:
        open_file = False
    if isinstance(indices, str):
        indices = load(indices, file_format="csv", delimiter=" ", use_eval=True)

    if start_idx is None:
        start_idx = 0
        if end_idx is None:
            end_idx = len(indices)
    if end_idx is None:
        end_idx = start_idx + 1
        only_one = True
    else:
        only_one = False

    end_idx -= 1
    ret = read_record(data_file, indices[start_idx][0], indices[end_idx][1], serialized)
    if only_one:
        ret = ret[0]
    if open_file:
        data_file.close()
    return ret


def load_record_indices(data_filenames):
    if isinstance(data_filenames, list):
        return [load_record_indices(data_filename) for data_filename in data_filenames]
    else:
        suffix = get_filename_suffix(data_filenames)
        if suffix in ["record", "record_episode"]:
            index_filename = get_index_filenames(data_filenames)
            return load(index_filename, file_format="csv", delimiter=" ", use_eval=True)
        else:
            return None


def get_index_filenames(data_filenames):
    if isinstance(data_filenames, list):
        return [get_index_filenames(data_filename) for data_filename in data_filenames]
    else:
        folder_name = osp.dirname(data_filenames)
        index_filename = osp.join(folder_name, replace_suffix(osp.basename(data_filenames), "index.txt"))
        return index_filename


def generate_index_from_record(filename):
    check_files_exist(filename)
    index_filename = get_index_filenames(filename)
    data_file = open(filename, "rb")
    index_file = open(index_filename, "w")
    while True:
        current = data_file.tell()
        try:
            byte_len = data_file.read(8)
            if len(byte_len) == 0:
                break
            data_file.read(4)
            proto_len = struct.unpack("q", byte_len)[0]
            data_file.read(proto_len)
            data_file.read(4)
            index_file.write(f"{current} {data_file.tell()}\n")
        except:
            get_logger().error("Failed to parse TFRecord.")
            break
    data_file.close()
    index_file.close()


def output_record(data: Union[list, DictArray, np.ndarray], data_filename, use_shuffle=False):
    """
    A dataset in deep learning should contains a series of data.
    """
    assert get_filename_suffix(data_filename) in ["record", "record_episode"]

    data_filename = osp.abspath(data_filename)
    index_filename = get_index_filenames(data_filename)
    mkdir_or_exist(osp.dirname(data_filename))

    num = len(data)
    print(f"The dataset contains {num} pieces of data.")
    print(f"The record_file will be stored at {data_filename}.")
    print(f"the index file will be stored at {index_filename}.")
    data_file = open(data_filename, "wb")
    index_file = open(index_filename, "w")
    indices = list(range(num))
    if use_shuffle:
        indices = shuffle(indices)
    for i in indices:
        item = data.slice(i).to_numpy().memory if isinstance(data, DictArray) else data[i]
        write_item_to_record(item, data_file, index_file)
    data_file.close()
    index_file.close()


def shuffle_reocrd(filename, output_name=None):
    if output_name is None:
        output_name = filename
    data = load_items_from_record(filename, indices=None, start_idx=None, end_idx=None, serialized=False)
    data = shuffle(data)
    data_file = open(output_name, "wb")
    index_file = open(get_index_filenames(output_name), "w")
    for item in data:
        write_item_to_record(item, data_file, index_file, serialized=True)


def shuffle_merge_records(data_filenames, num_shards=1, output_folder=None, dataset_name=None):
    """
    Assert the every seperate dataset can be load in memeory once.
    """
    start_time = time.time()
    if isinstance(data_filenames, str):
        data_filenames = [data_filenames]
    data_filenames = [osp.abspath(data_filename) for data_filename in data_filenames]

    suffix = np.unique([get_filename_suffix(_) for _ in data_filenames])
    if len(suffix) != 1:
        get_logger().error(f"Cannot merge data with different suffixes: {suffix}")
        exit(0)
    else:
        suffix = suffix[0]

    index_filenames = get_index_filenames(data_filenames)
    if output_folder is None:
        output_folder = osp.join(osp.dirname(data_filenames[0]), osp.basename(data_filenames[0]) + "_shards")
    mkdir_or_exist(output_folder)
    if dataset_name is None:
        dataset_name = osp.basename(output_folder)
    check_files_exist(data_filenames + index_filenames)
    indices = [
        [[eval(_) for _ in line.strip("\n").split(" ")] for line in open(index_filename, "r") if line != "\n"] for index_filename in index_filenames
    ]
    num_items = [len(_) for _ in indices]
    total_num = sum(num_items)
    num_files = len(data_filenames)
    num_shards, num_items_per_shard = split_num(total_num, num_shards)
    shard_index = [i for i in range(num_shards) for _ in range(num_items_per_shard[i])]
    assert len(shard_index) == total_num, f"Error: In correct split! {len(shard_index)} != {total_num}"
    shard_index = shuffle(shard_index)

    print(f"Num of datasets: {num_files}, num of items in all datasets: {total_num}.")
    print(f"We will use {num_shards} files to store the whole datasets.")

    if num_shards == 1:
        output_data_filenames = [osp.join(output_folder, f"{dataset_name}.{suffix}")]
    else:
        output_data_filenames = [osp.join(output_folder, f"{dataset_name}-{i}.{suffix}") for i in range(num_shards)]
    output_index_filenames = get_index_filenames(output_data_filenames)
    output_data_files = [open(output_data_filename, "wb") for output_data_filename in output_data_filenames]
    output_index_files = [open(output_index_filename, "w") for output_index_filename in output_index_filenames]
    cnt = 0
    for i, data_filename, index in zip(range(num_files), data_filenames, indices):
        data_file = open(data_filename, "rb")
        data = read_record(data_file)
        for j in range(len(index)):
            file_index = shard_index[cnt]
            cnt += 1
            item = data[j]
            write_item_to_record(item, output_data_files[file_index], output_index_files[file_index], serialized=True)
        data_file.close()
        print(f"Process file {i + 1}/{num_files}, file name: {data_filename}, num of items: {len(index)}, finish items: {cnt}/{total_num}!")
    for i in range(num_shards):
        output_data_files[i].close()
        output_index_files[i].close()
    print(f"Total time: {time.time() - start_time}")
    start_time = time.time()
    for i in range(num_shards):
        shuffle_reocrd(output_data_filenames[i])
    print(f"Shuffle time: {time.time() - start_time}")


"""
Trajectory-related Tools
"""


def merge_h5_trajectory(h5_files, output_name, num=-1):
    from h5py import File

    with File(output_name, "w") as f:
        index, meta = 0, None
        for h5_file in h5_files:
            h5 = File(h5_file, "r")
            h5_keys = set(h5.keys())
            if "meta" in h5_keys:
                h5_keys.remove("meta")
                if "meta" not in f.keys():
                    h5.copy("meta", f, "meta")

            for i in range(len(h5_keys)):
                if f"traj_{i}" in h5_keys:
                    h5.copy(f"traj_{i}", f, f"traj_{index}")
                    index += 1
                    if num != -1 and index >= num:
                        break
            if num != -1 and index >= num:
                break
        get_logger().info(f"Total number of trajectories {index}")


def convert_h5_trajectory_to_record(data, output_file, keep_episode=False, use_shuffle=False, keys=None):
    from .cache_utils import purify_items

    if isinstance(data, str):
        start_time = time.time()
        filename = data
        data = GDict.from_hdf5(data, wrapper=False)
        from .cache_utils import META_KEYS

        for key in META_KEYS:
            if key in data.memory:
                data.memory.pop(key)
        print(f"Loading {filename} time: {time.time() - start_time}")
    if keep_episode and get_filename_suffix(output_file) == "record":
        print("Replace the suffix to .record_episode when using episode mode")
        output_file = replace_suffix(output_file, "record_episode")
    # print(GDict(data).shape)
    start_time = time.time()
    data = [data[traj_key] for traj_key in data]
    data = [purify_items(_, keys, full=False, one_episode=True) for _ in data]

    if not keep_episode:
        data = DictArray.concat(data, axis=0)
    print(data.shape)
    output_record(data, output_file, use_shuffle)
    print(f"Output to {output_file} time: {time.time() - start_time}")


def convert_h5_trajectories_to_shard(folder, output_folder, num_shards=20, keep_episode=False, keys=None):
    folder = Path(folder)
    output_folder = Path(output_folder)
    tmp_folder = output_folder.parent / f"tmp_record_{get_time_stamp()}"

    mkdir_or_exist(str(tmp_folder))
    record_names = []
    for file in folder.glob("*.h5"):
        record_name = replace_suffix(file.name, "record")
        record_name = str(tmp_folder / record_name)
        convert_h5_trajectory_to_record(str(file), record_name, keep_episode=keep_episode, use_shuffle=True, keys=keys)
        record_names.append(record_name)
    mkdir_or_exist(str(output_folder))
    shuffle_merge_records(record_names, num_shards=num_shards, output_folder=output_folder)
    rmtree(str(tmp_folder))


"""
Dataset related utilities
"""


def train_test_split(items, ratio=[0.7, 0.1, 0.2]):
    assert len(ratio) == 3
    num = len(items)
    num_test = max(int(num * ratio[-2]), 1)
    num_val = max(int(num * ratio[-1]), 1)
    num_train = num - num_test - num_val
    assert num_train >= 1
    index = np.arange(num)
    np.random.shuffle(index)
    index = index.tolist()
    return (
        select_by_index(items, index[:num_train]),
        select_by_index(items, index[num_train : num_train + num_val]),
        select_by_index(items, index[num_train + num_val :]),
    )


def do_train_test_split(items, target_folder, ratio=[0.7, 0.1, 0.2]):
    train, val, test = train_test_split(items, ratio)

    train_folder = osp.join(target_folder, "train")
    val_folder = osp.join(target_folder, "val")
    test_folder = osp.join(target_folder, "test")

    mkdir_or_exist(train_folder)
    mkdir_or_exist(val_folder)
    mkdir_or_exist(test_folder)

    for file in train:
        symlink(file, osp.join(train_folder, osp.basename(file)), overwrite=True)
    for file in val:
        symlink(file, osp.join(val_folder, osp.basename(file)), overwrite=True)
    for file in test:
        symlink(file, osp.join(test_folder, osp.basename(file)), overwrite=True)
