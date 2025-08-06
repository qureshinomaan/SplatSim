import h5py
import pandas as pd

from .serialization import PickleProtocol


def try_to_open_hdf_trajectory(h5_file):
    try:
        h5 = h5py.File(h5_file, "r")
        key = sorted(h5.keys())[-1]
        h5.close()
        x = pd.read_hdf(h5_file, key)
    except:
        return False
    return True


def convert_hdf_with_pickle_4(h5_files):
    """
    Python 3.8 use pickle 5 which cannot be used in python 3.6.
    Convert the h5 files with pickle 3.
    """

    for h5_file in h5_files:
        try:
            h5 = h5py.File(h5_file, "r")
        except:
            print(f"Cannot open {h5_file}")
            continue
        keys = list(h5.keys())
        h5.close()
        objs = {}
        for key in keys:
            objs[key] = pd.read_hdf(h5_file, key=key)

        with PickleProtocol(4):
            for key in keys:
                objs[key].to_hdf(h5_file, key, mode="a")


def load_hdf(file_name, as_list=True):
    """
    load trajectory with pandas format
    """
    h5_file = h5py.File(file_name, "r")
    if as_list:
        return [pd.read_hdf(file_name, key) for key in sorted(h5_file.keys())]
    else:
        return {key: pd.read_hdf(file_name, key) for key in sorted(h5_file.keys())}


def hdf_to_dict_list(df):
    return df.reset_index().to_dict(orient="list")


def concat_hdf_trajectory(trajectories, required_keys=None):
    if len(trajectories) == 1:
        return trajectories[0]
    ret = {}
    length = 0
    for trajectory in trajectories:
        dict_list = hdf_to_dict_list(trajectory)
        effective_len = pd.notnull(trajectory.loc[:, "obs"]).sum()
        for key in dict_list:
            if required_keys is not None and key not in required_keys:
                continue
            if key not in ret:
                ret[key] = [None for i in range(length)]
            ret[key] += dict_list[key][:effective_len]
    return pd.DataFrame(ret)


def save_hdf_trajectory(trajectory, output_name):
    out_h5_file = pd.HDFStore(output_name, "w")
    index = 0
    for traj in trajectory:
        out_h5_file[f"traj_{index}"] = traj
        index += 1
    out_h5_file.close()


def merge_hdf_trajectory(h5_files, output_name):
    out_h5_file = pd.HDFStore(output_name, "w")
    index = 0
    for h5_file in h5_files:
        try:
            h5 = h5py.File(h5_file, "r")
        except:
            print(f"Cannot open {h5_file}")
            continue
        for traj_name in h5.keys():
            out_h5_file[f"traj_{index}"] = pd.read_hdf(h5_file, traj_name)
            index += 1
    out_h5_file.close()
    if index == 0:
        from shutil import rmtree

        print("Remove empty File")
        rmtree(output_name, ignore_errors=True)
