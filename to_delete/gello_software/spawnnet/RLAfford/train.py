# Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
import sys
from ast import arg
import numpy as np
import random

# appending paths for searching packages
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(BASE_DIR, "MARL_Module/envs/"))
sys.path.append(os.path.join(BASE_DIR, "Collision_Predictor_Module/CollisionPredictor/code"))
sys.path.append(os.path.join(BASE_DIR, "Collision_Predictor_Module/where2act/code"))

from MARL_Module.envs.utils.config import set_np_formatting, set_seed, get_args, parse_sim_params, load_cfg
from MARL_Module.envs.utils.parse_task import parse_task
from MARL_Module.envs.utils.process_sarl import *
from MARL_Module.envs.utils.process_marl import process_MultiAgentRL

import torch


def train():
    print("Algorithm: ", args.algo)
    agent_index = [[[0, 1, 2, 3, 4, 5]],
                   [[0, 1, 2, 3, 4, 5]]]

    if args.algo in ["mappo", "mappo_pc_pure", "happo", "hatrpo","maddpg","ippo"]: 
        # maddpg exists a bug now 
        args.task_type = "MultiAgent"

        task, env = parse_task(args, cfg, cfg_train, sim_params, agent_index, logdir)

        runner = process_MultiAgentRL(args, env=env, config=cfg_train, model_dir=args.model_dir)
        
        # test
        if args.model_dir != "":
            runner.eval(1000)
        else:
            runner.run()

    elif args.algo in ["ppo","ddpg","sac","td3","trpo","ppo_pc","ppo_pc_pure","sac_pc_pure"]:
        task, env = parse_task(args, cfg, cfg_train, sim_params, agent_index, logdir)

        sarl = eval('process_{}'.format(args.algo))(args, env, cfg_train, logdir)

        iterations = cfg_train["learn"]["max_iterations"]
        if args.max_iterations > 0:
            iterations = args.max_iterations

        sarl.run(num_learning_iterations=iterations, log_interval=cfg_train["learn"]["save_interval"])
    

    else:
        print("Unrecognized algorithm!\nAlgorithm should be one of: [happo, hatrpo, mappo, mappo_pc_pure, ippo, maddpg, sac, td3, trpo, ppo, ddpg, ppo_pc, ppo_pc_pure, sac_pc_pure]")


if __name__ == '__main__':
    set_np_formatting()
    args = get_args()
    cfg, cfg_train, logdir = load_cfg(args)
    sim_params = parse_sim_params(args, cfg, cfg_train)
    set_seed(cfg_train.get("seed", -1), cfg_train.get("torch_deterministic", False))
    train()
