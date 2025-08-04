import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions.normal import Normal
import numpy as np
from algorithms.utils.maniskill_learn.networks.backbones.pointnet import getPointNet


LOG_STD_MAX = 2
LOG_STD_MIN = -20


def mlp(sizes, activation, output_activation=nn.Identity):
    layers = []
    for j in range(len(sizes)-1):
        act = activation if j < len(sizes)-2 else output_activation
        layers += [nn.Linear(sizes[j], sizes[j+1]), act()]
    return nn.Sequential(*layers)




class SquashedGaussianMLPActor(nn.Module):

    def __init__(self, obs_dim, act_dim, hidden_sizes, activation, act_limit):
        super().__init__()
        self.net = mlp([obs_dim] + list(hidden_sizes), activation, activation)
        self.mu_layer = nn.Linear(hidden_sizes[-1], act_dim)
        self.log_std_layer = nn.Linear(hidden_sizes[-1], act_dim)
        self.act_limit = act_limit

    def forward(self, obs, deterministic=False, with_logprob=True, epsilon=1e-6):
        net_out = self.net(obs)
        mu = self.mu_layer(net_out)
        log_std = self.log_std_layer(net_out)
        log_std = torch.clamp(log_std, LOG_STD_MIN, LOG_STD_MAX)
        std = torch.exp(log_std)

        # Pre-squash distribution and sample
        pi_distribution = Normal(mu, std)
        if deterministic:
            # Only used for evaluating policy at test time.
            pi_action = mu
        else:
            pi_action = pi_distribution.rsample()

        if with_logprob:
            # Compute logprob from Gaussian, and then apply correction for Tanh squashing.
            # NOTE: The correction formula is a little bit magic. To get an understanding
            # of where it comes from, check out the original SAC paper (arXiv 1801.01290)
            # and look in appendix C. This is a more numerically-stable equivalent to Eq 21.
            # Try deriving it yourself as a (very difficult) exercise. :)
            logp_pi = (pi_distribution.log_prob(pi_action) - torch.log(1 - torch.tanh(pi_action).pow(2) + epsilon)).sum(axis=-1, keepdim=True)
            # logp_pi -= (2*(np.log(2) - pi_action - F.softplus(-2*pi_action))).sum(axis=1)
        else:
            logp_pi = None

        pi_action = torch.tanh(pi_action)
        pi_action = self.act_limit * pi_action

        return pi_action, logp_pi


class MLPQFunction(nn.Module):

    def __init__(self, obs_dim, act_dim, hidden_sizes, activation):
        super().__init__()
        self.q = mlp([obs_dim + act_dim] + list(hidden_sizes) + [1], activation)

    def forward(self, obs, act):
        q = self.q(torch.cat([obs, act], dim=-1))
        return q # Critical to ensure q has right shape.
        # return torch.squeeze(q, -1) # Critical to ensure q has right shape.

class MLPActorCritic(nn.Module):

    def __init__(self, observation_space, action_space,hidden_sizes=(256,256),
                 activation=nn.ELU):
        super().__init__()

        obs_dim = observation_space.shape[0]
        act_dim = action_space.shape[0]
        act_limit = action_space.high[0]

        # build policy and value functions
        self.pi = SquashedGaussianMLPActor(obs_dim, act_dim, hidden_sizes, activation, act_limit)
        self.q1 = MLPQFunction(obs_dim, act_dim, hidden_sizes, activation)
        self.q2 = MLPQFunction(obs_dim, act_dim, hidden_sizes, activation)

    def act(self, obs, deterministic=False):
        with torch.no_grad():
            a, _ = self.pi(obs, deterministic, False)
            return a.detach()
    
    def prepare_o(self, obs) :

        return obs

class MLPActorCriticPC(nn.Module):

    def __init__(self, observation_space, action_space, hidden_sizes=(256,256),
                 activation=nn.ELU, model_cfg=None, feature_dim=None):
        super().__init__()

        act_dim = action_space.shape[0]
        act_limit = action_space.high[0]

        self.feature_dim = feature_dim
        self.pc_dim = 3 + model_cfg["task_meta"]["mask_dim"]
        self.state_dim = model_cfg["task_meta"]["obs_dim"] - self.pc_dim
        self.contrastive = model_cfg["contrastive"]
        self.contrastive_m = model_cfg['contrastive_m']
        self.contrastive_loss_func = nn.CrossEntropyLoss()

        self.pointnet_layer = getPointNet({ # query encoder
            'input_feature_dim': self.pc_dim,
            'feat_dim': self.feature_dim
        })

        if self.contrastive :               # momentum encoder
            self.pointnet_layer_copy = getPointNet({
            'input_feature_dim': self.pc_dim,
            'feat_dim': self.feature_dim
            })
            for param in self.pointnet_layer_copy.parameters() :
                param.requires_grad = False
            self.W = nn.Parameter(torch.randn((self.feature_dim, self.feature_dim), requires_grad=True))

        # build policy and value functions
        self.pi = SquashedGaussianMLPActor(self.feature_dim+self.state_dim, act_dim, hidden_sizes, activation, act_limit)
        self.q1 = MLPQFunction(self.feature_dim+self.state_dim, act_dim, hidden_sizes, activation)
        self.q2 = MLPQFunction(self.feature_dim+self.state_dim, act_dim, hidden_sizes, activation)

    def act(self, obs, deterministic=False):
        with torch.no_grad():
            a, _ = self.pi(self.prepare_o(obs), deterministic, False)
            return a.detach()
    
    def prepare_o(self, obs, contrastive=False):

        pc_inp, state_inp = obs[..., :self.pc_dim], obs[..., 0, self.pc_dim:]
        pc_batch_shape = pc_inp.shape[:-2]
        pc_data_shape = pc_inp.shape[-2:]
        pc_out = self.pointnet_layer(pc_inp.view(-1, *pc_data_shape)).view(*pc_batch_shape, -1)
        inp = torch.cat((pc_out, state_inp), dim=-1)
        if contrastive :
            pc_copy_out = self.pointnet_layer_copy(pc_inp.view(-1, *pc_data_shape)).view(*pc_batch_shape, -1)
            return inp, pc_out, pc_copy_out
        else :
            return inp