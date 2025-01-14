import argparse
import os

os.environ["D4RL_SUPPRESS_IMPORT_ERROR"] = "1"
os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"

import os.path as osp
from multiprocessing import Process

import h5py

from mani_skill_learn.env import make_gym_env, ReplayMemory
from mani_skill_learn.utils.fileio import load_h5_as_dict_array, merge_h5_trajectory
from mani_skill_learn.utils.data import sample_element_in_dict_array, compress_size
from mani_skill_learn.utils.meta import get_total_memory, flush_print


def auto_fix_wrong_name(traj):
    for key in traj:
        if key in ['action', 'reward', 'done', 'env_level', 'next_env_level', 'next_env_state', 'env_state']:
            traj[key + 's'] = traj[key]
            del traj[key]
    return traj


tmp_folder_in_docker = '/tmp'


def convert_state_representation(keys, args, worker_id, main_process_id):
    env = make_gym_env(args.env_name, unwrapped=True, obs_mode=args.obs_mode)
    assert hasattr(env, 'get_obs'), f'env {env} does not contain get_obs'
    get_obs = env.get_obs

    cnt = 0
    output_file = osp.join(tmp_folder_in_docker, f'{worker_id}.h5')
    if worker_id == 0:
        flush_print(f'Save trajectory to {output_file}')
    output_h5 = h5py.File(output_file, 'w')
    input_h5 = h5py.File(args.traj_name, 'r')

    for j, key in enumerate(keys):
        trajectory = load_h5_as_dict_array(input_h5[key])
        trajectory = auto_fix_wrong_name(trajectory)
        env.reset(level=trajectory['env_levels'][0])
        length = trajectory['obs'].shape[0]

        if 'info_eval_info_success' in trajectory:
            if 'info_keep_threshold' not in trajectory:
                success = trajectory['info_eval_info_success'][-1]
            else:
                success = trajectory['info_eval_info_success'][-1]
                keep_threshold = trajectory['info_keep_threshold'][-1]
                success = success >= keep_threshold
        elif 'eval_info_success' in trajectory:
            success = trajectory['eval_info_success'][-1]
            keep_threshold = trajectory['keep_threshold'][-1]
            success = success >= keep_threshold
        else:
            flush_print(trajectory.keys(), 'No success info')
            raise Exception("")

        if not success:
            if worker_id == 0:
                flush_print(f'Worker {worker_id}, Skip {j + 1}/{len(keys)}, Choose {cnt}')
            continue
        replay = ReplayMemory(length)

        next_obs = None
        for i in range(length):
            if next_obs is None:
                env_state = sample_element_in_dict_array(trajectory['env_states'], i)
                env.set_state(env_state)
                obs = get_obs()
                obs = compress_size(obs)
            else:
                obs = next_obs
                # from mani_skill_learn.utils.data import get_shape_and_type
            # flush_print(get_shape_and_type(obs))
            # exit(0)

            next_env_state = sample_element_in_dict_array(trajectory['next_env_states'], i)
            env.set_state(next_env_state)
            next_obs = get_obs()
            next_obs = compress_size(next_obs)

            item_i = {
                'obs': obs,
                'next_obs': next_obs,
                'actions': trajectory['actions'][i],
                'dones': trajectory['dones'][i],
                'rewards': trajectory['rewards'][i],
            }
            mem = get_total_memory('G', False, init_pid=main_process_id)
            replay.push(**item_i)
            if worker_id == 0:
                flush_print(f'Convert Trajectory: choose{cnt + 1}, {j + 1}/{len(keys)}, Step {i + 1}/{length}, total mem:{mem}')
        group = output_h5.create_group(f'traj_{cnt}')
        cnt += 1
        replay.to_h5(group, with_traj_index=False)
    output_h5.close()
    flush_print(f'Finish using {output_file}')


def get_running_steps(num, n):
    assert num >= n
    min_steps = num // n
    running_steps = []
    for i in range(n):
        if i < num - min_steps * n:
            running_steps.append(min_steps + 1)
        else:
            running_steps.append(min_steps)
    assert sum(running_steps) == num
    return running_steps


def parse_args():
    parser = argparse.ArgumentParser(description='Convert the representation of the trajectory')
    # Configurations
    parser.add_argument('--env-name', default='OpenCabinetDrawer_1045_link_0-v0',
                        help='The name of the environment')
    parser.add_argument('--traj-name',
                        default='./debug_mani_skill/OpenCabinetDrawer_1045_link_0-v0/test/trajectory.h5',
                        help='The generated trajectory with some policies')
    parser.add_argument('--output-name',
                        default='./debug_mani_skill/OpenCabinetDrawer_1045_link_0-v0/test/trajectory_pcd.h5',
                        help='The generated trajectory with some policies')
    parser.add_argument('--max-num-traj', default=-1, type=int, help='The generated trajectory with some policies')
    parser.add_argument('--obs-mode', default='pointcloud', type=str, help='The mode of the observer')
    parser.add_argument('--num-procs', default=10, type=int, help='The mode of the observer')

    # Convert setting
    parser.add_argument('--add-random', default=False, action='store_true', help='Add random trajectory')
    args = parser.parse_args()
    args.traj_name = osp.abspath(args.traj_name)
    args.output_name = osp.abspath(args.output_name)
    return args


def main():
    os.makedirs(osp.dirname(args.output_name), exist_ok=True)

    with h5py.File(args.traj_name, 'r') as h5_file:
        keys = sorted(h5_file.keys())
    if args.max_num_traj < 0:
        args.max_num_traj = len(keys)
    args.max_num_traj = min(len(keys), args.max_num_traj)
    args.num_procs = min(args.num_procs, args.max_num_traj)
    keys = keys[:args.max_num_traj]
    running_steps = get_running_steps(len(keys), args.num_procs)
    flush_print(f'Num of trajs {len(keys)}', args.num_procs)
    processes = []
    from copy import deepcopy
    for i, x in enumerate(running_steps):
        p = Process(target=convert_state_representation, args=(deepcopy(keys[:x]), args, i, os.getpid()))
        keys = keys[x:]
        processes.append(p)
        p.start()

    for p in processes:
        p.join()
    files = []
    for worker_id in range(len(running_steps)):
        tmp_h5 = osp.join(tmp_folder_in_docker, f'{worker_id}.h5')
        files.append(tmp_h5)
    from shutil import rmtree
    rmtree(args.output_name, ignore_errors=True)
    merge_h5_trajectory(files, args.output_name)
    for file in files:
        rmtree(file, ignore_errors=True)


if __name__ == '__main__':
    args = parse_args()
    main()
