import numpy as np

import torch
import torch.nn as nn
from torch.distributions import MultivariateNormal
from algorithms.utils.model_3d import PointNet2Feature
from algorithms.utils.maniskill_learn.networks.backbones.pointnet import getPointNet, getNaivePointNet
from algorithms.utils.maniskill_learn.networks.backbones.pointnet import getPointNetWithInstanceInfo
import ipdb


class ActorCritic(nn.Module):

    def __init__(self, obs_shape, states_shape, actions_shape, initial_std, model_cfg, asymmetric=False):
        super(ActorCritic, self).__init__()

        self.asymmetric = asymmetric

        if model_cfg is None:
            actor_hidden_dim = [256, 256, 256]
            critic_hidden_dim = [256, 256, 256]
            activation = get_activation("selu")
        else:
            actor_hidden_dim = model_cfg['pi_hid_sizes']
            critic_hidden_dim = model_cfg['vf_hid_sizes']
            activation = get_activation(model_cfg['activation'])

        # Policy
        actor_layers = []
        actor_layers.append(nn.Linear(*obs_shape, actor_hidden_dim[0]))
        actor_layers.append(activation)
        for l in range(len(actor_hidden_dim)):
            if l == len(actor_hidden_dim) - 1:
                actor_layers.append(nn.Linear(actor_hidden_dim[l], *actions_shape))
            else:
                actor_layers.append(nn.Linear(actor_hidden_dim[l], actor_hidden_dim[l + 1]))
                actor_layers.append(activation)
        self.actor = nn.Sequential(*actor_layers)

        # Value function
        critic_layers = []
        if self.asymmetric:
            critic_layers.append(nn.Linear(*states_shape, critic_hidden_dim[0]))
        else:
            critic_layers.append(nn.Linear(*obs_shape, critic_hidden_dim[0]))
        critic_layers.append(activation)
        for l in range(len(critic_hidden_dim)):
            if l == len(critic_hidden_dim) - 1:
                critic_layers.append(nn.Linear(critic_hidden_dim[l], 1))
            else:
                critic_layers.append(nn.Linear(critic_hidden_dim[l], critic_hidden_dim[l + 1]))
                critic_layers.append(activation)
        self.critic = nn.Sequential(*critic_layers)

        # print(self.actor)
        # print(self.critic)

        # Action noise
        self.log_std = nn.Parameter(np.log(initial_std) * torch.ones(*actions_shape))

        # Initialize the weights like in stable baselines
        actor_weights = [np.sqrt(2)] * len(actor_hidden_dim)
        actor_weights.append(0.01)
        critic_weights = [np.sqrt(2)] * len(critic_hidden_dim)
        critic_weights.append(1.0)
        self.init_weights(self.actor, actor_weights)
        self.init_weights(self.critic, critic_weights)

    @staticmethod
    def init_weights(sequential, scales):
        [torch.nn.init.orthogonal_(module.weight, gain=scales[idx]) for idx, module in
         enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))]

    def forward(self):
        raise NotImplementedError

    def act(self, observations, states):
        actions_mean = self.actor(observations)

        covariance = torch.diag(self.log_std.exp() * self.log_std.exp())
        distribution = MultivariateNormal(actions_mean, scale_tril=covariance)

        actions = distribution.sample()
        actions_log_prob = distribution.log_prob(actions)

        if self.asymmetric:
            value = self.critic(states)
        else:
            value = self.critic(observations)

        return actions.detach(), actions_log_prob.detach(), value.detach(), actions_mean.detach(), self.log_std.repeat(actions_mean.shape[0], 1).detach()

    def act_inference(self, observations):
        actions_mean = self.actor(observations)
        return actions_mean.detach()

    def evaluate(self, observations, states, actions, contrastive=False):
        actions_mean = self.actor(observations)

        covariance = torch.diag(self.log_std.exp() * self.log_std.exp())
        distribution = MultivariateNormal(actions_mean, scale_tril=covariance)

        actions_log_prob = distribution.log_prob(actions)
        entropy = distribution.entropy()

        if self.asymmetric:
            value = self.critic(states)
        else:
            value = self.critic(observations)

        return actions_log_prob, entropy, value, actions_mean, self.log_std.repeat(actions_mean.shape[0], 1), 0

class MyNet(nn.Module):
    def __init__(self, point_net, fc, split) :
        super(MyNet, self).__init__()
        self.point_net = point_net
        self.fc = fc
        self.split = split
    
    def split_data(self, input) :

        return input[:, :, :self.split], input[:, 0, self.split:]
    
    def forward(self, inp) :
        x1, x2 = self.split_data(inp)
        y1 = self.point_net(x1)
        # x2 = x2.repeat(1, 3, 1).view(-1, 39*3)
        inp2 = torch.cat((x2, y1), dim=1)
        y2 = self.fc(inp2)
        return y2


class ActorCriticPC(nn.Module):

    def __init__(self, obs_shape, states_shape, actions_shape, initial_std, model_cfg, asymmetric=False):
        super(ActorCriticPC, self).__init__()

        self.asymmetric = asymmetric
        self.feature_dim = model_cfg["feature_dim"]
        self.pc_dim = 3 + model_cfg["task_meta"]["mask_dim"]
        self.state_dim = model_cfg["task_meta"]["obs_dim"] - self.pc_dim
        self.contrastive = model_cfg["contrastive"]
        self.contrastive_m = model_cfg['contrastive_m']
        self.contrastive_loss_func = nn.CrossEntropyLoss()

        if model_cfg is None:
            actor_hidden_dim = [256, 256, 256]
            critic_hidden_dim = [256, 256, 256]
            activation = get_activation("selu")
        else:
            actor_hidden_dim = model_cfg['pi_hid_sizes']
            critic_hidden_dim = model_cfg['vf_hid_sizes']
            activation = get_activation(model_cfg['activation'])

        # Policy
        # obs_shape need to be (point_num, 3+feat_dim)


        # for more info about the dual net, refer CURL: Contrastive RL
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


        # self.pointnet_layer = PointNet2Feature({'input_feature_dim': obs_shape[1]-3, 'feat_dim': self.feature_dim})
        # self.pointnet_layer = getPointNetWithInstanceInfo({
        #     'mask_dim': model_cfg["task_meta"]["mask_dim"],
        #     'pc_dim': 3,
        #     'state_dim': obs_shape[1]-3-model_cfg["task_meta"]["mask_dim"],
        #     'output_dim': self.feature_dim
        # })

        actor_layers = []
        actor_layers.append(nn.Linear(self.feature_dim + self.state_dim, actor_hidden_dim[0]))
        actor_layers.append(activation)
        for l in range(len(actor_hidden_dim)):
            if l == len(actor_hidden_dim) - 1:
                actor_layers.append(nn.Linear(actor_hidden_dim[l], *actions_shape))
            else:
                actor_layers.append(nn.Linear(actor_hidden_dim[l], actor_hidden_dim[l + 1]))
                actor_layers.append(activation)
        self.actor1 = nn.Sequential(*actor_layers)

        # Value function
        critic_layers = []
        if self.asymmetric:
            critic_layers.append(nn.Linear(*states_shape, critic_hidden_dim[0]))
        else:
            critic_layers.append(nn.Linear(self.feature_dim + self.state_dim, critic_hidden_dim[0]))
        critic_layers.append(activation)
        for l in range(len(critic_hidden_dim)):
            if l == len(critic_hidden_dim) - 1:
                critic_layers.append(nn.Linear(critic_hidden_dim[l], 1))
            else:
                critic_layers.append(nn.Linear(critic_hidden_dim[l], critic_hidden_dim[l + 1]))
                critic_layers.append(activation)
        self.critic1 = nn.Sequential(*critic_layers)

        self.actor = MyNet(self.pointnet_layer, self.actor1, self.pc_dim)
        self.critic = MyNet(self.pointnet_layer, self.critic1, self.pc_dim)

        # print(self.actor)
        # print(self.critic)

        # Action noise
        self.log_std = nn.Parameter(np.log(initial_std) * torch.ones(*actions_shape))

        # Initialize the weights like in stable baselines
        # actor_weights = [np.sqrt(2)] * len(actor_hidden_dim)
        # actor_weights.append(0.01)
        # critic_weights = [np.sqrt(2)] * len(critic_hidden_dim)
        # critic_weights.append(1.0)
        # self.init_weights(self.actor, actor_weights)
        # self.init_weights(self.critic, critic_weights)


    @staticmethod
    def init_weights(sequential, scales):
        [torch.nn.init.orthogonal_(module.weight, gain=scales[idx]) for idx, module in
         enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))]

    def forward(self):
        raise NotImplementedError

    def act(self, observations, states):

        pc_inp, state_inp = self.actor.split_data(observations)
        pc_out = self.actor.point_net(pc_inp)
        inp = torch.cat((pc_out, state_inp), dim=1)

        actions_mean = self.actor.fc(inp)

        covariance = torch.diag(self.log_std.exp() * self.log_std.exp())
        distribution = MultivariateNormal(actions_mean, scale_tril=covariance)

        actions = distribution.sample()
        actions_log_prob = distribution.log_prob(actions)

        if self.asymmetric:
            value = self.critic(states)
        else:
            value = self.critic.fc(inp)

        return actions.detach(), actions_log_prob.detach(), value.detach(), actions_mean.detach(), self.log_std.repeat(actions_mean.shape[0], 1).detach()

    def act_inference(self, observations):
        
        pc_inp, state_inp = self.actor.split_data(observations)
        pc_out = self.actor.point_net(pc_inp)
        inp = torch.cat((pc_out, state_inp), dim=1)

        actions_mean = self.actor.fc(inp)

        return actions_mean.detach()

    def evaluate(self, observations, states, actions, contrastive):

        pc_inp, state_inp = self.actor.split_data(observations)
        pc_out = self.actor.point_net(pc_inp)
        inp = torch.cat((pc_out, state_inp), dim=1)

        contrastive_loss = 0
        if contrastive :
            # do contrastive learning
            for theta_q, theta_k in zip(self.pointnet_layer.parameters(), self.pointnet_layer_copy.parameters()) :
                theta_k = theta_k * self.contrastive_m + theta_q.detach() * (1-self.contrastive_m)
            # print(theta_q, theta_k)
            label = torch.arange(0, observations.shape[0], device=observations.device).long()
            pc_out_aug = self.pointnet_layer_copy(pc_inp)
            sim_mat = pc_out @ self.W @ pc_out_aug.T
            contrastive_loss = self.contrastive_loss_func(sim_mat, label)

        actions_mean = self.actor.fc(inp)

        covariance = torch.diag(self.log_std.exp() * self.log_std.exp())
        distribution = MultivariateNormal(actions_mean, scale_tril=covariance)

        actions_log_prob = distribution.log_prob(actions)
        entropy = distribution.entropy()

        if self.asymmetric:
            value = self.critic(states)
        else:
            value = self.critic.fc(inp)

        return actions_log_prob, entropy, value, actions_mean, self.log_std.repeat(actions_mean.shape[0], 1), contrastive_loss


def get_activation(act_name):
    if act_name == "elu":
        return nn.ELU()
    elif act_name == "selu":
        return nn.SELU()
    elif act_name == "relu":
        return nn.ReLU()
    elif act_name == "crelu":
        return nn.ReLU()
    elif act_name == "lrelu":
        return nn.LeakyReLU()
    elif act_name == "tanh":
        return nn.Tanh()
    elif act_name == "sigmoid":
        return nn.Sigmoid()
    else:
        print("invalid activation function!")
        return None
