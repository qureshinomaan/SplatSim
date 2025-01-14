"""
Soft Actor-Critic Algorithms and Applications
    https://arxiv.org/abs/1812.05905
Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor
    https://arxiv.org/abs/1801.01290
Recurrent Model-Free RL Can Be a Strong Baseline for Many POMDPs
    https://arxiv.org/abs/2110.05038
Soft Actor-Critic for Discrete Action Settings
    https://arxiv.org/abs/1910.07207
"""
import numpy as np
from copy import deepcopy
import torch
import torch.nn as nn
import torch.nn.functional as F

from pyrl.networks import build_all, build_actor_critic, build_target_network
from pyrl.utils.data import GDict
from pyrl.utils.torch import BaseAgent, hard_update, soft_update, build_optimizer, disable_gradients
from pyrl.utils.augmentations import build_data_augmentations
from ..builder import MFRL


@MFRL.register_module()
class SAC(BaseAgent):
    def __init__(
        self,
        actor_cfg,
        critic_cfg,
        env_params,
        batch_size=128,
        gamma=0.99,
        reward_scale=1,
        update_coeff=0.005,
        alpha=0.2,
        alpha_optim_cfg=None,
        automatic_alpha_tuning=True,
        target_entropy=None,
        ignore_dones=False,
        use_episode_dones=False,
        target_update_interval=1,
        actor_update_interval=1,
        shared_backbone=False,
        shared_target_backbone=None,
        detach_actor_feature=False,
        target_smooth=0.90,  # For discrete SAC
        pre_process=None,
    ):
        super(SAC, self).__init__()
        self.is_discrete = env_params["is_discrete"]

        self.gamma = gamma
        self.update_coeff = update_coeff
        self.alpha = alpha
        self.reward_scale = reward_scale
        self.ignore_dones = ignore_dones
        self.batch_size = batch_size
        self.target_update_interval = target_update_interval
        self.actor_update_interval = actor_update_interval
        self.automatic_alpha_tuning = automatic_alpha_tuning
        self.shared_backbone = shared_backbone
        self.detach_actor_feature = detach_actor_feature

        self.obs_processor = build_data_augmentations(pre_process)
        actor_cfg, critic_cfg = deepcopy([actor_cfg, critic_cfg])
        actor_optim_cfg, critic_optim_cfg = actor_cfg.pop("optim_cfg"), critic_cfg.pop("optim_cfg")
        actor_cfg.update(env_params)
        critic_cfg.update(env_params)

        self.actor, self.critic = build_actor_critic(actor_cfg, critic_cfg, shared_backbone)
        self.actor_optim = build_optimizer(self.actor, actor_optim_cfg)
        self.critic_optim = build_optimizer(self.critic, critic_optim_cfg)
        self.use_episode_dones = use_episode_dones

        shared_target_backbone = shared_backbone if shared_target_backbone is None else shared_target_backbone
        self.target_critic = build_target_network(critic_cfg, self.critic, self.actor, shared_target_backbone)

        # for param in self.target_critic.parameters():
        #     print(param.shape, param.requires_grad)

        self.is_recurrent = self.actor.is_recurrent

        self.log_alpha = nn.Parameter(torch.ones(1, requires_grad=True))
        self.log_alpha.data *= float(np.log(np.float32(alpha)))

        if target_entropy is None:
            action_shape = env_params["action_shape"]
            if env_params["is_discrete"]:
                # Use label smoothing to get the target entropy.
                n = np.prod(action_shape)
                explore_rate = (1 - target_smooth) / (n - 1)
                self.target_entropy = -(target_smooth * np.log(target_smooth) + (n - 1) * explore_rate * np.log(explore_rate))
                self.log_alpha = nn.Parameter(torch.tensor(np.log(0.1), requires_grad=True))
                # self.target_entropy = np.log(action_shape) * target_smooth
            else:
                self.target_entropy = -np.prod(action_shape)
        else:
            self.target_entropy = target_entropy
        if self.automatic_alpha_tuning:
            self.alpha = self.log_alpha.exp().item()
        self.alpha_optim = build_optimizer(self.log_alpha, alpha_optim_cfg)

    def update_parameters(self, memory, updates):
        sampled_batch = memory.sample(self.batch_size).to_torch(device=self.device, non_blocking=True)
        sampled_batch = self.process_obs(sampled_batch)
        if self.use_episode_dones:
            sampled_batch["dones"] = sampled_batch["episode_dones"]
        next_obs, actions, prev_actions = sampled_batch["next_obs"], sampled_batch["actions"], sampled_batch["prev_actions"]

        with torch.no_grad():
            if self.is_recurrent:
                first_frame = GDict([sampled_batch["obs"], sampled_batch["prev_actions"] * 0]).slice(slice(0, 1), axis=1, wrapper=False)
                current_infos = [sampled_batch["next_obs"], sampled_batch["actions"]]
                next_obs, actions = GDict.concat([first_frame, current_infos], axis=1)
                prev_actions = actions[:, :-1]

            with self.actor.no_sync():
                next_actions, neg_logp = self.actor(next_obs, prev_actions=actions, mode="max-entropy", rnn_mode="base")

            action_kwargs = dict(actions_prob=next_actions) if self.is_discrete else dict(actions=next_actions)

            with self.target_critic.no_sync():
                q_next_target = self.target_critic(next_obs, prev_actions=actions, **action_kwargs)

            min_q_next_target = torch.min(q_next_target, dim=-1, keepdim=True).values

            min_q_next_target += self.alpha * neg_logp
            min_q_next_target = min_q_next_target[:, 1:] if self.is_recurrent else min_q_next_target

            if self.ignore_dones:
                q_target = sampled_batch["rewards"] * self.reward_scale + self.gamma * min_q_next_target
            else:
                q_target = sampled_batch["rewards"] * self.reward_scale + (1 - sampled_batch["dones"].float()) * self.gamma * min_q_next_target
            q_target = q_target.repeat_interleave(q_next_target.shape[-1], dim=-1)

        q = self.critic(sampled_batch["obs"], sampled_batch["actions"], prev_actions=prev_actions)
        critic_loss = F.mse_loss(q, q_target) * q_target.shape[-1]

        with torch.no_grad():
            abs_critic_error = torch.abs(q - q_target).max().item()
        self.critic_optim.zero_grad()
        critic_loss.backward()
        self.critic_optim.step()
        with torch.no_grad():
            critic_grad = self.critic.grad_norm

        if self.shared_backbone:
            self.critic_optim.zero_grad()

        ret = {
            "sac/critic_loss": critic_loss.item(),
            "sac/max_critic_abs_err": abs_critic_error,
            "sac/alpha": self.alpha,
            "sac/q": torch.min(q, dim=-1).values.mean().item(),
            "sac/q_target": torch.mean(q_target).item(),
            "sac/target_entropy": self.target_entropy,
            "sac/critic_grad": critic_grad,
            "sac/grad_steps": 1,
        }

        if updates % self.actor_update_interval == 0:
            pi, neg_logp = self.actor(
                sampled_batch["obs"],
                mode="max-entropy",
                prev_actions=prev_actions,
                save_feature=self.shared_backbone,
                detach_visual=self.detach_actor_feature,
            )[:2]
            entropy_term = neg_logp.mean()

            visual_feature = self.actor.backbone.pop_attr("saved_visual_feature")
            if visual_feature is not None:
                visual_feature = visual_feature.detach()

            if self.is_discrete:
                q = self.critic(sampled_batch["obs"], prev_actions=prev_actions, visual_feature=visual_feature, detach_value=True).min(-2).values
                q_pi = (q * pi).sum(-1)
                with torch.no_grad():
                    q_match_rate = (pi.argmax(-1) == q.argmax(-1)).float().mean().item()
            else:
                q_pi = self.critic(sampled_batch["obs"], actions=pi, prev_actions=prev_actions, visual_feature=visual_feature)
                q_pi = torch.min(q_pi, dim=-1, keepdim=True).values
            actor_loss = -(q_pi.mean() + self.alpha * entropy_term)
            self.actor_optim.zero_grad()
            actor_loss.backward()
            self.actor_optim.step()
            with torch.no_grad():
                actor_grad = self.actor.grad_norm

            if self.automatic_alpha_tuning:
                alpha_loss = self.log_alpha.exp() * (entropy_term - self.target_entropy).detach()
                self.alpha_optim.zero_grad()
                alpha_loss.backward()
                self.alpha_optim.step()
                self.alpha = self.log_alpha.exp().item()
            else:
                alpha_loss = torch.tensor(0.0).to(self.device)

            ret_actor = {
                "sac/actor_loss": actor_loss.item(),
                "sac/alpha_loss": alpha_loss.item(),
                "sac/entropy": entropy_term.item(),
                "sac/actor_grad": actor_grad,
            }
            ret.update(ret_actor)

        if updates % self.target_update_interval == 0:
            soft_update(self.target_critic, self.critic, self.update_coeff)


        if self.is_discrete:
            ret["sac/q_match_rate"] = q_match_rate

        return ret
