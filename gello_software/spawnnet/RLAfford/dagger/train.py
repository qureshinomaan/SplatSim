import os

import hydra
import numpy as np
import wandb
from einops import rearrange, repeat
from hydra.core.hydra_config import HydraConfig
from omegaconf import DictConfig, OmegaConf, ListConfig
from tqdm import tqdm

from RLAfford.dagger.prepare_env import prepare_isaacgym_env_expert
from RLAfford.dagger.mp_wrapper import MultiprocEnvWrapper

from gpu_info import vulkan_cuda_idxes

def isaacgym_obs_wrapper(info):
    ret = {}
    rgb = repeat(info['rgb'], 'b v h w c -> b v c h w')
    depth = repeat(info['depth'], 'b v h w c -> b v c h w')
    ret['rgb'] = rgb
    ret['depth'] = depth
    ret['state'] = info['proprio']

    return ret

def collect_rollout(env, expert, encoder, policy, expert_rollout=False, buffer=None):
    """
    Collect rollout using the expert policy.
    """
    import torch
    from simple_bc.utils import torch_utils
    obses, actions = [], []
    total_success = torch.zeros(env.num_environments, device=expert.device)
    env.task.eval()
    _ = env.reset()
    obs, _, _, info = env.step(actions=torch.zeros([env.num_environments, env.num_actions], device=expert.device))
    for _ in tqdm(range(expert.max_episode_length), desc='rollout'):
        wrapped_obs = isaacgym_obs_wrapper(info)
        obses.append(wrapped_obs)

        with torch.no_grad():
            # Adding a frame for rgb and depth
            if not expert_rollout:
                wrapped_obs = wrapped_obs.copy()
                wrapped_obs['rgb'] = repeat(wrapped_obs['rgb'], 'b v c h w -> b f v c h w', f=1)
                wrapped_obs['depth'] = repeat(wrapped_obs['depth'], 'b v c h w -> b f v c h w', f=1)
                wrapped_obs['state'] = repeat(wrapped_obs['state'], 'b d -> b f d', f=1)

                pred_action, _ = policy(encoder(wrapped_obs))
                pred_action = pred_action[:, 0]  # One-step
                
            action = expert.actor_critic.act_inference(obs)                       
            action = torch.clamp(action, -1, 1)  # Raw output is very large, but it is clipped in the env
            actions.append(action)
        if expert_rollout:
            obs, _, _, info = env.step(actions=action)
        else:
            obs, _, _, info = env.step(actions=pred_action)
        total_success += info['successes'].to(expert.device)
    if buffer is not None:
        buffer.add_traj(obses, actions)
    rgb_frames = [torch_utils.to_cpu(obs['rgb']) for obs in obses]
    rgb_frames = rearrange(np.stack(rgb_frames, axis=0), 't b v c h w -> v b t h w c')[0]  # Take first view
    return {'avg_success': torch.mean(total_success).item(),
            'std_success': torch.std(total_success).item(),
            'rgb_frames': rgb_frames,
            'all_success': torch_utils.to_cpu(total_success).numpy(),
            }

@hydra.main(config_path="../../conf", version_base="1.3")
def main(cfg: DictConfig):    
    # cuda_devices are completely enumerated and indices are exactly matched to the GPUs
    num_gpus = cfg.num_gpus
    mode = os.environ['HYDRA_LAUNCHER']
    cuda_gpu_idxes, vulkan_gpu_idxes = vulkan_cuda_idxes(mode, num_gpus)
    
    # noinspection PyUnresolvedReferences
    from isaacgym import gymapi

    work_dir = HydraConfig.get().runtime.output_dir
    setup(cfg)
        
    assert type(cfg.isaacgym_task) == str
    cfg.isaacgym_arg_str = os.environ.get('ISAACGYM_ARG_STR', '') + f' --isaacgym_task={cfg.isaacgym_task}'
    
    if num_gpus > 1:
        env = MultiprocEnvWrapper(num_gpus=num_gpus, cuda_gpu_idxes=cuda_gpu_idxes, vulkan_gpu_idxes=vulkan_gpu_idxes, isaacgym_str=cfg.isaacgym_arg_str)
        expert = env.expert
    else:
        env, expert = prepare_isaacgym_env_expert(isaacgym_arg_str=cfg.isaacgym_arg_str, 
                                                  cuda_gpu_idx=cuda_gpu_idxes[0], vulkan_gpu_idx=vulkan_gpu_idxes[0])

    from simple_bc._interfaces.encoder import Encoder
    from simple_bc._interfaces.policy import Policy
    from simple_bc.dataset.replay_online import ReplayBuffer
    from simple_bc.utils import log_utils

    encoder = Encoder.build_encoder(cfg.encoder).to(expert.device)
    policy = Policy.build_policy(
        encoder.out_shape,
        cfg.policy,
        cfg.encoder).to(expert.device)  # Updated shape

    OmegaConf.save(config=cfg, f=os.path.join(work_dir, 'config.yaml'))

    buffer = ReplayBuffer(aug_cfg=cfg.dataset.aug_cfg)

    from simple_bc.constants import ISAACGYM_TASK
    task_constants = ISAACGYM_TASK[cfg.isaacgym_task]
    train_iteration = task_constants['train_iteration']
    save_freq = task_constants['save_freq']

    optimizer = setup_optimizer(cfg.optimizer_cfg, encoder, policy)
    scheduler = setup_lr_scheduler(optimizer, cfg.scheduler_cfg, train_iteration)

    if not cfg.debug:
        log_utils.init_wandb(cfg)

    # Pick ckpt based on the average of the last 5 epochs
    metric_logger = log_utils.MetricLogger(delimiter=" ")

    encoder.eval()
    policy.eval()
    
    while buffer.num_trajs < cfg.warmup_traj:
        info = collect_rollout(env, expert, encoder, policy, expert_rollout=True, buffer=buffer)
        print('Warmup success rate: ', info['avg_success'])

    all_train_success = []
    for iteration in metric_logger.log_every(range(int(train_iteration)), 1, ''):
        encoder.train()
        policy.train()
                
        train_metrics = run_one_epoch(
            cfg, encoder, policy, buffer, optimizer, scheduler, clip_grad=cfg.clip_grad)

        encoder.eval()
        policy.eval()
        
        rollout_info = collect_rollout(env, expert, encoder, policy, buffer=buffer)

        train_metrics['train/lr'] = optimizer.param_groups[0]['lr']
        train_metrics['train/rollout_success'] = rollout_info['avg_success']
        all_train_success.append(rollout_info['avg_success'])
        metric_logger.update(**train_metrics)
        if not cfg.debug:
            wandb.log(train_metrics, step=iteration)

        from simple_bc.utils.visualization_utils import make_grid_video_from_numpy
        make_grid_video_from_numpy(rollout_info['rgb_frames'], 4, f"{work_dir}/videos/train_rollout_{iteration}.mp4")

        if iteration % save_freq == 0:
            encoder.save(f"{work_dir}/encoder_{iteration}.ckpt")
            policy.save(f"{work_dir}/policy_{iteration}.ckpt")

        if not cfg.debug:
            wandb.log(
                {'train/num_trajs': buffer.num_trajs,
                 'train/num_timesteps': buffer.num_timesteps}, step=iteration)

    # Save best train success
    with open(f"{work_dir}/best_train_success.txt", 'w') as f:
        f.write('Best train success: %.4f' % (max(all_train_success)))
    encoder.save(f"{work_dir}/encoder_final.ckpt")
    policy.save(f"{work_dir}/policy_final.ckpt")

    if not cfg.debug:
        wandb.finish()

    if mode == 'basic':
        env.close() # put here to avoid segfault
    
    if mode == 'slurm':
        from RLAfford.dagger.slurm_launch_eval import launch_eval_slurm
        launch_eval_slurm(work_dir)

# noinspection PyUnresolvedReferences
def setup_optimizer(optim_cfg, encoder, policy):
    """
    Setup the optimizer. Return the optimizer.
    """
    from torch import optim
    from simple_bc.utils import log_utils, torch_utils
    optimizer = eval(optim_cfg.type)

    encoder_trainable_params = torch_utils.get_named_trainable_params(encoder)
    # Print size of trainable parameters
    print('Encoder trainable parameters:', sum(p.numel() for (name, p) in encoder_trainable_params) / 1e6, 'M')
    print('Policy trainable parameters:', sum(p.numel() for p in policy.parameters()) / 1e6, 'M')
    if len(encoder_trainable_params) > 0:
        return optimizer(list(encoder.parameters()) + list(policy.parameters()), **optim_cfg.params)
    else:
        return optimizer(list(policy.parameters()), **optim_cfg.params)

# noinspection PyUnresolvedReferences
def setup_lr_scheduler(optimizer, scheduler_cfg, train_iteration):
    import torch.optim as optim
    import torch.optim.lr_scheduler as lr_scheduler
    from simple_bc.utils.lr_scheduler import CosineAnnealingLRWithWarmup
    sched = eval(scheduler_cfg.type)
    if sched is None:
        return None
    if 'T_max' in scheduler_cfg.params:
        scheduler_cfg.params['T_max'] = train_iteration
    return sched(optimizer, **scheduler_cfg.params)

def run_one_epoch(cfg, encoder, policy, buffer, optimizer, scheduler=None, clip_grad=None):
    """
    Optimize the policy. Return a dictionary of the loss and any other metrics.
    """
    import torch
    from torch import nn
    import gdict

    running_loss, running_mse, running_abs, tot_items = 0, 0, 0, 0
    
    loss_fn = nn.MSELoss(reduction='none')

    for _ in tqdm(range(cfg.updates_per_iteration), desc='Training'):
        batch = buffer.sample(cfg.batch_size)
        obs, act = batch["obs"], batch["actions"]
        obs = gdict.GDict(obs).cuda(device='cuda')
        act = act.to(device='cuda')

        optimizer.zero_grad()
        pred, _ = policy(encoder(obs))  # pred: (B, H, A)

        loss = loss_fn(pred, act).mean()

        loss.backward()

        if clip_grad is not None:
            torch.nn.utils.clip_grad_norm_(policy.parameters(), 0.5)

        optimizer.step()
        running_mse += ((pred - act) ** 2).sum(0).mean().item()
        running_abs += (torch.abs(pred - act)).sum(0).mean().item()
        running_loss += loss.item() * act.shape[0]
        tot_items += act.shape[0]

    out_dict = {"train/mse": running_mse / tot_items,
                "train/abs": running_abs / tot_items,
                "train/loss": running_loss / tot_items,}
    if scheduler is not None:
        scheduler.step()

    return out_dict

def setup(cfg):
    import warnings
    warnings.simplefilter("ignore")
    
    import torch
    torch.multiprocessing.set_start_method("spawn")
        
    from simple_bc.utils.log_utils import set_random_seed
    set_random_seed(cfg.seed)
    torch.manual_seed(cfg.seed)

if __name__ == "__main__":
    main()

