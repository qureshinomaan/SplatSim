# noinspection PyUnresolvedReferences
from isaacgym import gymapi
import copy
import os
import re
from glob import glob

import click
import numpy as np

from natsort import natsorted
from omegaconf import OmegaConf

from RLAfford.dagger.train import collect_rollout, prepare_isaacgym_env_expert
from simple_bc._interfaces.encoder import Encoder
from simple_bc._interfaces.policy import Policy

from gpu_info import vulkan_cuda_idxes

# default argument is policy_path
@click.command()
@click.argument('exp_folder', type=click.Path(exists=True))
@click.option('--chunk_id', type=int)
@click.option('--num_chunks', type=int)
@click.option('--n_eval', default=5, help='Number of rollouts to evaluate')
@click.option('--mode', default='slurm', help='slurm or basic.')
def main(exp_folder, chunk_id, num_chunks, n_eval, mode):
    cuda_gpu_idxes, vulkan_gpu_idxes = vulkan_cuda_idxes(mode, 1)
        
    cfg_path = os.path.join(exp_folder, 'config.yaml')
    train_cfg = OmegaConf.load(cfg_path)
    
    # Create env
    isaacgym_arg_str = train_cfg.isaacgym_arg_str
    
    isaacgym_arg_str = re.sub(r'--num_envs=\d+', '', isaacgym_arg_str)
    isaacgym_arg_str = re.sub(r'--num_objs=\d+', '', isaacgym_arg_str)
    isaacgym_arg_str += ' --eval_only=True'

    env, expert = prepare_isaacgym_env_expert(
        isaacgym_arg_str,
        chunk_id=chunk_id, num_chunks=num_chunks,
        cuda_gpu_idx=cuda_gpu_idxes[0], vulkan_gpu_idx=vulkan_gpu_idxes[0],
        mode='val',) # One gpu

    # Find all policies in a folder
    policy_files = glob(os.path.join(exp_folder, 'policy_*.ckpt'))
    policy_files = natsorted(policy_files)
    
    encoder = Encoder.build_encoder(train_cfg.encoder).to(expert.device)
    policy = Policy.build_policy(
        encoder.out_shape,
        train_cfg.policy,
        train_cfg.encoder).to(expert.device)  # Updated shape
    
    eval_dir = os.path.join(exp_folder, 'eval')
     
    csv_name = os.path.join(eval_dir, f'result_{chunk_id+1}_{num_chunks}.csv')
    all_success, all_success_std, names = [], [], []

    for policy_file in policy_files:
        print('Evaluating policy: ', policy_file)
        encoder_file = policy_file.replace('policy_', 'encoder_')
        policy.load(policy_file)
        encoder.load(encoder_file)
        
        encoder.eval()
        policy.eval()

        name = os.path.basename(policy_file).replace('.ckpt', '').replace('policy_', '')

        ckpt_success = []
        for i in range(n_eval):
            rollout_info = collect_rollout(env, expert, encoder, policy, expert_rollout=False)
            ckpt_success.append(rollout_info['all_success'])

        print(f"Success rate per asset: {np.mean(np.array(ckpt_success), axis=0)}")

        from simple_bc.utils.visualization_utils import make_grid_video_from_numpy
        make_grid_video_from_numpy(rollout_info['rgb_frames'], 4, f"{eval_dir}/train_rollout_{name}_chunk_{chunk_id+1}_{num_chunks}.mp4")
        all_success.append(np.mean(ckpt_success))
        # TODO Need to computer variance differently
        all_success_std.append(np.std(np.array(ckpt_success).flatten()))
        names.append(copy.copy(name))

        with open(csv_name, 'w') as f:
            f.write('Policy, Success Rate, Success Std\n')
            for i, (success, std, n) in enumerate(zip(all_success, all_success_std, names)):
                f.write(f'{n}, {success}, {std}\n')
        
        from mp_eval_launcher import aggregate_results
        aggregate_results(eval_dir)
        
    env.close()

if __name__ == "__main__":
    main()
