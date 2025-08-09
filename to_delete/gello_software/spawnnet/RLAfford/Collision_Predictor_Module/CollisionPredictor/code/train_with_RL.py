"""
    Train the full model
"""

from json import load
import os
import time
import sys
import shutil
import random
from time import strftime
from argparse import ArgumentParser
import numpy as np
import torch
import torch.utils.data
import torch.nn.functional as F
import cp_utils
import torch.nn as nn
import ipdb
from torch.utils.tensorboard import SummaryWriter

class linear_decay_scheduler():

    def __init__(self, optimizer, begin, decay) :

        self.optimizer = optimizer
        self.step_size = begin
        self.decay = decay

    def step(self) :

        for param_group in self.optimizer.param_groups:
            param_group['lr'] = self.step_size
        
        self.step_size -= self.decay
        if self.step_size <= 0:
            self.step_size = 0.0
    
    def state_dict(self) :

        return {"step_size": self.step_size, "decay": self.decay}
    
    def load_state_dict(self, state_dict) :

        self.step_size = state_dict["step_size"]
        self.decay = state_dict["decay"]

class CollisionPredictor():

    def __init__(self, cfg, log_dir):
        self.multi_gpu = cfg["cp"]["multi_gpu"]
        if self.multi_gpu == True :
            self.output_device_id = int(cfg["cp"]["output_device_id"].split(':')[1])

            self.output_device = "cuda:" + str(self.output_device_id)
            self.device_ids = cfg["cp"]["device_ids"]
        else :
            if cfg["cp"]["output_device_id"] == "cpu" :
                self.output_device_id = None
                self.output_device = "cpu"
            else :    # cuda:x
                print("########", cfg["cp"]["output_device_id"], "$$$$$$$$$$$")
                self.output_device_id = int(cfg["cp"]["output_device_id"].split(':')[1])
                self.output_device = "cuda:" + str(self.output_device_id)
        
        print("CP: training on", self.output_device)
        
        self.network = self.get_network(cfg)
        self.network_opt = self.get_opt(cfg, self.network)
        self.network_lr_scheduler = self.get_lr_scheduler(cfg, self.network_opt)
        self.log_dir = log_dir
        self.cfg = cfg
        self.writer = SummaryWriter(log_dir=self.log_dir)

    def get_network(self, cfg):
        model_def = cp_utils.get_model_module('model')
        # create models
        network = model_def.Network(cfg['cp']['input_feat'], cfg['cp']['feat_dim'])  # default = 128
        network.to(self.output_device)
        if self.multi_gpu :
            network = nn.DataParallel(network, self.device_ids, self.output_device_id)
        return network

    def get_opt(self, cfg, network):
        # create optimizers
        network_opt = torch.optim.Adam(network.parameters(), lr=cfg['cp']['lr'], weight_decay=float(cfg['cp']['weight_decay']))
        cp_utils.optimizer_to_device(network_opt, self.output_device)
        return network_opt

    def get_lr_scheduler(self, cfg, network_opt):
        # learning rate scheduler
        # network_lr_scheduler = linear_decay_scheduler(network_opt, cfg['cp']['lr'], cfg['cp']['lr_decay_by']/cfg['cp']['lr_decay_every'])
        network_lr_scheduler = torch.optim.lr_scheduler.StepLR(network_opt, step_size=cfg['cp']['lr_decay_every'], gamma=cfg['cp']['lr_decay_by'])
        return network_lr_scheduler

    def save_checkpoint(self, path):
        with torch.no_grad():
            print('Saving checkpoint ...... ')
            save_dict = {
                "network": self.network.state_dict(),
                "optimizer": self.network_opt.state_dict(),
                "lr_scheduler": self.network_lr_scheduler.state_dict()
            }
            torch.save(save_dict, path)
        
    def load_checkpoint(self, path):
        load_dict = torch.load(path, map_location=self.output_device)
        self.network.load_state_dict(load_dict["network"])
        self.network_opt.load_state_dict(load_dict["optimizer"])
        self.network_lr_scheduler.load_state_dict(load_dict["lr_scheduler"])

    def pred_one_batch(self, data, success_rate, target=None, num_train=0):

        if not self.multi_gpu :
            data = data.to(self.output_device)
            success_rate = success_rate.to(self.output_device)
            if target != None :
                target = target.to(self.output_device)
        # data: B*N*4  # xyz+mask
        # success_rate: B
        # target: B*N
        if target == None :
            self.network.eval()
        else:
            self.network.train()
        
        # forward through the network
        if target == None :
            with torch.no_grad() :
                output = self.network(data)  # B*N
        else :
            output = self.network(data)  # B*N
        
        # loss backward
        if target != None :
            gt_train = target.view(target.shape[0], target.shape[1])[:num_train, :]
            gt_val = target.view(target.shape[0], target.shape[1])[num_train:, :]

            pred_train = output[:num_train, :]
            pred_val = output[num_train:, :]
            tot_train_loss = (pred_train - gt_train) ** 2    
            tot_val_loss = (pred_val - gt_val) ** 2    
            train_loss = tot_train_loss.mean(dim=1)        # num_train
            val_loss = tot_val_loss.mean(dim=1)        # num_val
        
            success_rate_train = success_rate[:num_train]
            success_rate_val = success_rate[num_train:]

            # for each type of loss, compute avg loss per batch
            train_eff_loss = (train_loss*success_rate_train).mean()
            val_eff_loss = (val_loss*success_rate_val).mean()

            # compute total loss
            train_total_loss = train_eff_loss * self.cfg['cp']['loss_weight_action_score']
            val_total_loss = val_eff_loss * self.cfg['cp']['loss_weight_action_score']
            if num_train<target.shape[0]:
                cp_info_dict = {
                    'CP learning rate': self.network_opt.state_dict()['param_groups'][0]['lr'],
                    'CP training loss': train_loss.mean().item(),
                    'CP validate loss': val_loss.mean().item()
                }
            else:
                cp_info_dict = {
                    'CP learning rate': self.network_opt.state_dict()['param_groups'][0]['lr'],
                    'CP training loss': train_loss.mean().item(),
                }
            # optimize one step
            self.network_opt.zero_grad()
            train_total_loss.backward()
            self.network_opt.step()
            return output.detach(), cp_info_dict
        
        return output.detach()

    ## TODO:
    # tensor board
    # improve output to terminal
    # save checkpoint
    # improve training speed


# if __name__ == '__main__':

    
#     cfg={'cp':{'feat_dim': 128,
#     'batch_size': 10,
#     'lr': 0.01,
#     'weight_decay': 1e-5,
#     'lr_decay_by': 0.9,
#     'lr_decay_every': 5000,
#     'device': 1,
#     'loss_weight_action_score': 100.0,
#     'exp_dir': 'logs'}
#     }

#     data = torch.zeros((10, 1024, 4), device=0)
#     network = get_network(cfg)

#     network_opt = get_opt(cfg)

#     network_lr_scheduler = get_lr_scheduler(cfg)

#     train_one_batch(cfg, network, network_opt, network_lr_scheduler, 0, data)

