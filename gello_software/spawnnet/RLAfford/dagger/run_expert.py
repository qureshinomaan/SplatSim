# noinspection PyUnresolvedReferences
from isaacgym import gymapi
import click
import os
from tqdm import tqdm

from MARL_Module.envs.utils.process_sarl import *
from dagger.eval import prepare_isaacgym_env_expert
import numpy as np
@click.command()
@click.option('--isaacgym_arg_str', type=str)
@click.option('--chunk_id', type=int)
@click.option('--num_chunks', type=int)
@click.option('--n_eval', default=10, help='Number of rollouts to evaluate')
@click.option('--mode', type=str)
def train(isaacgym_arg_str, chunk_id, num_chunks, n_eval, mode):
    if 'open_drawer' in isaacgym_arg_str:
        task_name = 'open_drawer'
    elif 'open_door' in isaacgym_arg_str:
        task_name = 'open_door'
    else:
        raise NotImplementedError


    env, expert = prepare_isaacgym_env_expert(
        isaacgym_arg_str,
        chunk_id=chunk_id, num_chunks=num_chunks,
        cuda_gpu_idx=0, vulkan_gpu_idx=0,
        mode='val', expert_run=True, expert_eval_mode=mode)  # One gpu
    print(env, expert)

    import torch

    total_success = torch.zeros(env.num_environments, device=expert.device)
    env.task.eval()
    for _ in range(n_eval):
        _ = env.reset()
        obs, _, _, info = env.step(actions=torch.zeros([env.num_environments, env.num_actions], device=expert.device))
        for i in tqdm(range(expert.max_episode_length), desc='rollout'):
            with torch.no_grad():
                action = expert.actor_critic.act_inference(obs)
                action = torch.clamp(action, -1, 1)  # Raw output is very large, but it is clipped in the env
                obs, _, _, info = env.step(actions=action)
            total_success += info['successes'].to(expert.device)
    np.set_printoptions(precision=3, suppress=True)
    success = total_success / n_eval
    success_list = np.array(success.cpu().numpy()).tolist()
    save_dir = './data/corl_2023/'
    save_name = '{}_success_rate_{}_{}_{}.txt'.format(task_name, mode, chunk_id, num_chunks)
    with open(os.path.join(save_dir, save_name), 'w') as f:
        f.write(str(success_list))

    # expert
    # sarl = eval('process_{}'.format(args.algo))(args, env, cfg_train, logdir)
    # sarl.test(args.expert_policy)
    #
    # sarl.eval_round = n_eval

    # env.task.eval()
    # env.reset()
    # total_success = torch.zeros((sarl.train_env_num + sarl.val_env_num), device=sarl.device)
    #
    # with torch.no_grad():
    #     for r in tqdm(range(sarl.eval_round)):
    #         current_obs = env.reset()
    #         if args.use_image_obs:
    #             env.start_record()
    #         for i in range(sarl.max_episode_length):
    #             actions = sarl.actor_critic.act_inference(current_obs)
    #             next_obs, rews, dones, infos = env.step(actions)
    #             current_obs.copy_(next_obs)
    #             total_success += infos["successes"].to(sarl.device)
    #         if args.use_image_obs:
    #             env.end_record(f'./expert_{r}.mp4')
    #
    # train_success = total_success[:sarl.train_env_num].mean() / sarl.eval_round
    # test_success = total_success[sarl.train_env_num:].mean() / sarl.eval_round
    #
    # train_success = train_success.cpu().item()
    # test_success = test_success.cpu().item()
    #
    # print("Training set average success:    ", train_success)
    # print("Testing set average success:     ", test_success)
    #
    # print("Training set success list:")
    # for x in total_success[:sarl.train_env_num] / sarl.eval_round:
    #     print(x.cpu().item(), end=' ')
    #
    # print("\n\nTesting set success list:")
    # for x in total_success[sarl.train_env_num:] / sarl.eval_round:
    #     print(x.cpu().item(), end=' ')
    #
    # env.task.train()
    #
    # asset_name_list = env.task.cabinet_asset_name_list
    # high_succ_list, succ_list = [], []
    # for i, asset_name in enumerate(asset_name_list):
    #     if total_success[i] / sarl.eval_round > 0.7:
    #         high_succ_list.append(asset_name)
    #         succ_list.append( total_success[i].item() / sarl.eval_round)
    #
    # save_dir = './data/isaacgym_expert/'
    # # Dump asset name and the corresponding success rate
    # import csv
    # with open(os.path.join(save_dir, f'{args.isaacgym_task}.csv'), 'w') as f:
    #     writer = csv.writer(f)
    #     writer.writerow(['asset_name', 'success_rate'])
    #     for i, (asset_name, succ) in enumerate(zip(high_succ_list, succ_list)):
    #         writer.writerow([asset_name, succ])

if __name__ == '__main__':
    train()
