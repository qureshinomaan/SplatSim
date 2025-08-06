"""
Image Augmentation Is All You Need: Regularizing Deep Reinforcement Learning from Pixels
    https://arxiv.org/abs/2004.13649
SVEA: Stabilized Q-Value Estimation under Data Augmentation
   https://arxiv.org/abs/2107.00644
"""

import torch
import torch.nn.functional as F
import numpy as np

from pyrl.utils.data import GDict
from pyrl.utils.torch import soft_update
from pyrl.utils.augmentations import build_data_augmentations

from .sac import SAC
from ..builder import MFRL


@MFRL.register_module()
class DrQ(SAC):
    def __init__(self, num_aug=2, obs_aug=None, svea=False, inference_aug=None, *args, **kwargs):
        super(DrQ, self).__init__(*args, **kwargs)
        if svea:
            assert num_aug == 1, "SVEA only needs num_aug=1"

        self.num_aug, self.svea = num_aug, svea
        if self.svea:
            assert self.num_aug == 1, "In the paper, they only use num_aug == 1!"
        self.obs_aug = build_data_augmentations(obs_aug)
        self.inference_aug = self.obs_aug if inference_aug == "same" else build_data_augmentations(inference_aug)

    @torch.no_grad()
    def forward(self, obs, **kwargs):
        if self.inference_aug is not None:
            obs = self.inference_aug(GDict(obs).to_torch(device=self.device, wrapper=False))
            # print(GDict(obs).shape)

            # obs_ = GDict(obs).to_numpy(wrapper=False)
            # from pyrl.utils.visualization import visualize_pcd

            # visualize_pcd(obs_["xyz"][0].transpose(1, 0), obs_["rgb"][0].transpose(1, 0) / 255.0, show_frame=True)

        return super().forward(obs, **kwargs)

    def update_parameters(self, memory, updates):
        sampled_batch = memory.sample(self.batch_size).to_torch(device=self.device, non_blocking=True)
        if self.use_episode_dones:
            sampled_batch["dones"] = sampled_batch["episode_dones"]

        with torch.no_grad():
            # aug_batch = sampled_batch.repeat(self.num_aug, 0, wrapper=False)
            aug_batch = GDict(sampled_batch).memory  # Shallow copy

            # Every operation is repeat_interleave [B, num_aug] = [B * num_aug]
            if not self.svea:
                # DrQ only use the augmented obs for
                aug_batch["obs"] = self.obs_aug(GDict(sampled_batch["obs"]).repeat(self.num_aug, 0, wrapper=False))
                aug_batch["actions"] = torch.repeat_interleave(sampled_batch["actions"], self.num_aug, dim=0)

                aug_batch["next_obs"] = self.obs_aug(GDict(sampled_batch["next_obs"]).repeat(self.num_aug, 0, wrapper=False))
                for key in ["rewards", "dones"]:
                    aug_batch[key] = torch.repeat_interleave(sampled_batch[key], self.num_aug, dim=0)
            else:
                # SEVA use the original obs + augmented obs
                aug_batch["obs"] = self.obs_aug(GDict(sampled_batch["obs"]).repeat(self.num_aug, 0, wrapper=False))
                aug_batch["obs"] = GDict.stack([aug_batch["obs"], sampled_batch["obs"]], axis=1).merge_axes([0, 1], wrapper=False)

                aug_batch["actions"] = torch.repeat_interleave(sampled_batch["actions"], self.num_aug + 1, dim=0)

            next_actions, neg_logp = self.actor(aug_batch["next_obs"], mode="max-entropy", rnn_mode="base")

            action_kwargs = dict(actions_prob=next_actions) if self.is_discrete else dict(actions=next_actions)
            q_next_target = self.target_critic(aug_batch["next_obs"], **action_kwargs)
            min_q_next_target = torch.min(q_next_target, dim=-1, keepdim=True).values
            min_q_next_target += self.alpha * neg_logp

            if self.ignore_dones:
                q_target = aug_batch["rewards"] + self.gamma * min_q_next_target
            else:
                q_target = aug_batch["rewards"] + (1 - aug_batch["dones"].float()) * self.gamma * min_q_next_target

            if not self.svea:
                q_target = q_target.reshape(len(sampled_batch), self.num_aug).mean(1, keepdim=True) # [B, 1]

            q_target = torch.repeat_interleave(q_target, self.num_aug + int(self.svea), dim=0)  # DrQ [B * num_aug, 1], SVEA [B * (num_aug + 1), 1]
            q_target = q_target.repeat(1, q_next_target.shape[-1])  # DrQ [B * num_aug, num_q], SVEA [B * (num_aug + 1), num_q]

        q = self.critic(aug_batch["obs"], aug_batch["actions"])  # [B * (num_aug + 1), num_q]
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
            "drq/critic_loss": critic_loss.item(),
            "drq/max_critic_abs_err": abs_critic_error,
            "drq/alpha": self.alpha,
            "drq/q": torch.min(q, dim=-1).values.mean().item(),
            "drq/q_target": torch.mean(q_target).item(),
            "drq/target_entropy": self.target_entropy,
            "drq/critic_grad": critic_grad,
            "drq/grad_steps": 1,
        }

        if updates % self.actor_update_interval == 0:
            obs = sampled_batch["obs"] if self.svea else GDict(aug_batch["obs"]).split_axis(0, [self.batch_size, -1]).slice(0, 1, wrapper=False)
            pi, neg_logp = self.actor(
                obs,
                mode="max-entropy",
                save_feature=self.shared_backbone,
                detach_visual=self.detach_actor_feature,
            )[:2]
            entropy_term = neg_logp.mean()

            visual_feature = self.actor.backbone.pop_attr("saved_visual_feature")
            if visual_feature is not None:
                visual_feature = visual_feature.detach()

            if self.is_discrete:
                q = self.critic(obs, visual_feature=visual_feature, detach_value=True).min(-2).values
                q_pi = (q * pi).sum(-1)
                with torch.no_grad():
                    q_match_rate = (pi.argmax(-1) == q.argmax(-1)).float().mean().item()
            else:
                q_pi = self.critic(obs, actions=pi, visual_feature=visual_feature)
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
                "drq/actor_loss": actor_loss.item(),
                "drq/alpha_loss": alpha_loss.item(),
                "drq/entropy": entropy_term.item(),
                "drq/actor_grad": actor_grad,
            }
            ret.update(ret_actor)

        if updates % self.target_update_interval == 0:
            soft_update(self.target_critic, self.critic, self.update_coeff)

        if self.is_discrete:
            ret["drq/q_match_rate"] = q_match_rate
        # if updates % 100 == 0:
        #     print(updates, ret)
        return ret
