# Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from ast import arg
import os
import sys
import yaml

from isaacgym import gymapi
from isaacgym import gymutil

import numpy as np
import random
import torch


def set_np_formatting():
    np.set_printoptions(edgeitems=30, infstr='inf',
                        linewidth=4000, nanstr='nan', precision=2,
                        suppress=False, threshold=10000, formatter=None)


def warn_task_name():
    raise Exception(
        "Unrecognized task!")

def warn_algorithm_name():
    raise Exception(
                "Unrecognized algorithm!\nAlgorithm should be one of: [ppo, happo, hatrpo, mappo]")


def set_seed(seed, torch_deterministic=False):
    if seed == -1 and torch_deterministic:
        seed = 42
    elif seed == -1:
        seed = np.random.randint(0, 10000)
    print("Setting seed: {}".format(seed))

    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    os.environ['PYTHONHASHSEED'] = str(seed)
    torch.cuda.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)

    if torch_deterministic:
        # refer to https://docs.nvidia.com/cuda/cublas/index.html#cublasApi_reproducibility
        os.environ['CUBLAS_WORKSPACE_CONFIG'] = ':4096:8'
        torch.backends.cudnn.benchmark = False
        torch.backends.cudnn.deterministic = True
        torch.set_deterministic(True)
    else:
        torch.backends.cudnn.benchmark = True
        torch.backends.cudnn.deterministic = False

    return seed


def retrieve_cfg(args, use_rlg_config=False):

    log_dir = None
    algo_cfg = None
    task_cfg = None


    #TODO: add config files of sac, td3
    # 这里的设计有点不合理 可以修正
    print("LLLLLLLLL")
    print(args.task)
    if args.task == "OneFrankaCabinet":
        log_dir = os.path.join(args.logdir, "franka_cabinet/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_cabinet.yaml"
    elif args.task == "OneFrankaCabinetPCPure":
        log_dir = os.path.join(args.logdir, "franka_cabinet_PC_pure/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_cabinet_PC_pure.yaml"
    elif args.task == "OneFrankaCabinetPCPartial":
        log_dir = os.path.join(args.logdir, "franka_cabinet_PC_partial/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_cabinet_PC_partial.yaml"
    elif args.task == "OneFrankaCabinetPCPartialCPMap":
        log_dir = os.path.join(args.logdir, "franka_cabinet_PC_partial_cp_map/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_cabinet_PC_partial_cp_map.yaml"
    elif args.task == "OneFrankaCabinetPCPartialCPState":
        log_dir = os.path.join(args.logdir, "franka_cabinet_PC_partial_cp_state/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_cabinet_PC_partial_cp_state.yaml"
    elif args.task == "OneFrankaCabinetPCPartialIntimeMap":
        log_dir = os.path.join(args.logdir, "franka_cabinet_PC_partial_intime_map/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_cabinet_PC_partial_intime_map.yaml"
    elif args.task == "OneFrankaCabinetPCPartialPureMap":
        log_dir = os.path.join(args.logdir, "franka_cabinet_PC_partial_pure_map/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_cabinet_PC_partial_pure_map.yaml"
    elif args.task == "OneFrankaChair":
        log_dir = os.path.join(args.logdir, "franka_chair/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_chair.yaml"
    elif args.task == "OneFrankaChairPCPartial":
        log_dir = os.path.join(args.logdir, "franka_chair_PC_partial/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_chair_PC_partial.yaml"
    elif args.task == "OneFrankaCabinetPCWhere2act":
        log_dir = os.path.join(args.logdir, "franka_cabinet_PC_where2act/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_cabinet_PC_partial_cp_map.yaml"
    elif args.task == "OneFrankaGrasp":
        log_dir = os.path.join(args.logdir, "franka_grasp/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_grasp_state.yaml"
    elif args.task == "OneFrankaGraspPCPartial" :
        log_dir = os.path.join(args.logdir, "franka_grasp_PC_Partial/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_grasp_PC_partial_cloud.yaml"
    elif args.task == "OneFrankaCabinetRealWorld" :
        log_dir = os.path.join(args.logdir, "franka_cabinet_real_world/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_cabinet_state_open_handle_real_world.yaml"
    elif args.task == "PAPRaw":
        log_dir = os.path.join(args.logdir, "PAPRaw/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/pap_raw.yaml"
    elif args.task == "PAPRaw":
        log_dir = os.path.join(args.logdir, "PAPRaw/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/pap_raw.yaml"
    elif args.task == "PushStapler":
        log_dir = os.path.join(args.logdir, "PushStapler/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/push_stapler.yaml"
    elif args.task == "OpenPot":
        log_dir = os.path.join(args.logdir, "OpenPot/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/open_pot.yaml"
    elif args.task == "PAPPartial":
        log_dir = os.path.join(args.logdir, "PAPPartial/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/pap_partial.yaml"
    elif args.task == "PAPA2O":
        log_dir = os.path.join(args.logdir, "PAPA2O/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/pap_a2o.yaml"
    elif args.task == "TwoFrankaChair" :
        log_dir = os.path.join(args.logdir, "two_franka_chair/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_chair_state_push.yaml"
    elif args.task == "TwoFrankaChairPCPartial" :
        log_dir = os.path.join(args.logdir, "two_franka_chair_pc_partial/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_chair_PC_partial_cloud_push.yaml"
    elif args.task == "TwoFrankaChairPCPartialCPMap" :
        log_dir = os.path.join(args.logdir, "two_franka_chair_pc_partial_cp_map/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_chair_PC_partial_cp_map_push.yaml"
    elif args.task == "TwoFrankaChairPCPartialMultiAgent" :
        log_dir = os.path.join(args.logdir, "two_franka_chair_pc_partial_multi_agent/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_chair_PC_partial_multi_agent_push.yaml"
    elif args.task == "TwoFrankaChairRealWorld" :
        log_dir = os.path.join(args.logdir, "two_franka_chair_realworld/{}/{}".format(args.algo, args.algo))
        algo_cfg = "cfg/{}/config.yaml".format(args.algo)
        task_cfg = "cfg/franka_chair_state_push_real_world.yaml"
    else:
        warn_task_name()
    if args.task_config != None :
        task_cfg = args.task_config
    if args.algo_config != None :
        algo_cfg = args.algo_config
    algo_cfg = 'RLAfford/{}'.format(algo_cfg)
    task_cfg = 'RLAfford/{}'.format(task_cfg)
    return log_dir, algo_cfg, task_cfg


def parse_sim_params(args, cfg, cfg_train):
    # initialize sim
    sim_params = gymapi.SimParams()
    sim_params.dt = 1./60.
    sim_params.num_client_threads = args.slices

    if args.physics_engine == gymapi.SIM_FLEX:
        if args.device != "cpu":
            print("WARNING: Using Flex with GPU instead of PHYSX!")
        sim_params.flex.shape_collision_margin = 0.01
        sim_params.flex.num_outer_iterations = 4
        sim_params.flex.num_inner_iterations = 10
    elif args.physics_engine == gymapi.SIM_PHYSX:
        sim_params.physx.solver_type = 1
        sim_params.physx.num_position_iterations = 4
        sim_params.physx.num_velocity_iterations = 0
        sim_params.physx.num_threads = 4
        sim_params.physx.use_gpu = args.use_gpu
        sim_params.physx.num_subscenes = args.subscenes
        sim_params.physx.max_gpu_contact_pairs = 8 * 1024 * 1024

    sim_params.use_gpu_pipeline = args.use_gpu_pipeline
    sim_params.physx.use_gpu = args.use_gpu

    # if sim options are provided in cfg, parse them and update/override above:
    if "sim" in cfg:
        gymutil.parse_sim_config(cfg["sim"], sim_params)

    # Override num_threads if passed on the command line
    if args.physics_engine == gymapi.SIM_PHYSX and args.num_threads > 0:
        sim_params.physx.num_threads = args.num_threads
    return sim_params


def load_cfg(args, use_rlg_config=False, expert_run=False):
    with open(os.path.join(os.getcwd(), args.cfg_train), 'r') as f:
        cfg_train = yaml.load(f, Loader=yaml.SafeLoader)

    with open(os.path.join(os.getcwd(), args.cfg_env), 'r') as f:
        cfg = yaml.load(f, Loader=yaml.SafeLoader)

    cfg["task"]["useTaskId"] = args.multitask

    # Override number of environments if passed on the command line
    if not expert_run:
        if args.eval_only:
            cfg["env"]["numTrain"] = 0
            cfg["env"]["asset"]["cabinetAssetNumTrain"] = 0
            cfg["env"]["asset"]["trainAssets"] = {}
        else:
            cfg["env"]["numVal"] = 0
            cfg["env"]["asset"]["cabinetAssetNumVal"] = 0
            cfg["env"]["asset"]["testAssets"] = {}


    if args.num_envs > 0:
        cfg["env"]["numTrain"] = args.num_envs
        cfg["env"]["numVal"] = 0

    if args.num_envs_val > 0:
        cfg["env"]["numVal"] = args.num_envs_val

    if args.num_objs > 0:
        cfg["env"]["asset"]["cabinetAssetNumTrain"] = args.num_objs
        cfg["env"]["asset"]["cabinetAssetNumVal"] = 0
        cfg["env"]["asset"]["assetNumTrain"] = args.num_objs
        cfg["env"]["asset"]["assetNumVal"] = 0

    if args.num_objs_val > 0 :
        cfg["env"]["asset"]["cabinetAssetNumVal"] = args.num_objs_val
        cfg["env"]["asset"]["assetNumVal"] = args.num_objs_val

    if args.episode_length > 0:
        cfg["env"]["episodeLength"] = args.episode_length

    if args.contact_buffer_size != None :
        cfg["env"]["contactBufferSize"] = args.contact_buffer_size

    if args.no_mpr :
        cfg["cp"]["max_point_reward"] = 0
    if args.no_mpo :
        cfg["cp"]["max_point_observation"] = False

    if args.cp_lr>=0 :
        cfg["cp"]["lr"] = args.cp_lr
    if args.mpr>=0 :
        cfg["cp"]["max_point_reward"] = args.mpr
    if args.lr>=0 :
        cfg_train["learn"]["lr_upper"] = args.lr
        cfg_train["learn"]["lr_lower"] = min(args.lr, float(cfg_train["learn"]["lr_lower"]))

    cfg["name"] = args.task
    cfg["headless"] = args.headless

    # Set physics domain randomization
    if "task" in cfg:
        if "randomize" not in cfg["task"]:
            cfg["task"]["randomize"] = args.randomize
        else:
            cfg["task"]["randomize"] = args.randomize or cfg["task"]["randomize"]
    else:
        cfg["task"] = {"randomize": False}

    logdir = args.logdir
    if use_rlg_config:

        # Set deterministic mode
        if args.torch_deterministic:
            cfg_train["params"]["torch_deterministic"] = True

        exp_name = cfg_train["params"]["config"]['name']

        if args.experiment != 'Base':
            if args.metadata:
                exp_name = "{}_{}_{}_{}".format(args.experiment, args.task_type, args.device, str(args.physics_engine).split("_")[-1])

                if cfg["task"]["randomize"]:
                    exp_name += "_DR"
            else:
                exp_name = args.experiment

        # Override config name
        cfg_train["params"]["config"]['name'] = exp_name

        if args.resume > 0:
            cfg_train["params"]["load_checkpoint"] = True

        if args.checkpoint != "Base":
            cfg_train["params"]["load_path"] = args.checkpoint

        # Set maximum number of training iterations (epochs)
        if args.max_iterations > 0:
            cfg_train["params"]["config"]['max_epochs'] = args.max_iterations

        cfg_train["params"]["config"]["num_actors"] = cfg["env"]["numEnvs"]

        seed = cfg_train["params"].get("seed", -1)
        if args.seed is not None:
            seed = args.seed
        cfg["seed"] = seed
        cfg_train["params"]["seed"] = seed

        cfg["args"] = args
    else:

        # Set deterministic mode
        if args.torch_deterministic:
            cfg_train["torch_deterministic"] = True

        # Override seed if passed on the command line
        if args.seed is not None:
            cfg_train["seed"] = args.seed

        log_id = args.logdir
        if args.experiment != 'Base':
            if args.metadata:
                log_id = args.logdir + "_{}_{}_{}_{}".format(args.experiment, args.task_type, args.device, str(args.physics_engine).split("_")[-1])
                if cfg["task"]["randomize"]:
                    log_id += "_DR"
            else:
                log_id = args.logdir + "_{}".format(args.experiment)

        logdir = os.path.realpath(log_id)
        # os.makedirs(logdir, exist_ok=True)
        # print(args.test)
        if args.test :
            cfg_train["learn"]["test"] = True
        cfg_train["learn"]["contrastive_learning"] = args.contrastive
        if args.contrastive_m != None :
            cfg_train["learn"]["contrastive_m"] = args.contrastive_m

        cp_device = args.cp_device
        if cp_device != "default" :
            cfg["cp"]["output_device_id"] = cp_device
        elif "cp" in cfg :
            cfg["cp"]["output_device_id"] = "cuda:" + str(cfg["cp"]["output_device_id"])

        if args.visualize_pc :
            cfg["env"]["visualizePointcloud"] = True

    return cfg, cfg_train, logdir


def get_args(benchmark=False, use_rlg_config=False, isaacgym_arg_str=None):
    from simple_bc.constants import DATA_DIR
    custom_parameters = [
        {"name": "--test", "action": "store_true", "default": False,
            "help": "Run trained policy, no training"},
        {"name": "--play", "action": "store_true", "default": False,
            "help": "Run trained policy, the same as test, can be used only by rl_games RL library"},
        {"name": "--resume", "type": int, "default": 0,
            "help": "Resume training or start testing from a checkpoint"},
        {"name": "--checkpoint", "type": str, "default": "Base",
            "help": "Path to the saved weights, only for rl_games RL library"},
        {"name": "--headless", "action": "store_true", "default": False,
            "help": "Force display off at all times"},
        {"name": "--horovod", "action": "store_true", "default": False,
            "help": "Use horovod for multi-gpu training, have effect only with rl_games RL library"},
        {"name": "--task", "type": str, "default": "Humanoid",
            "help": "Can be BallBalance, Cartpole, CartpoleYUp, Ant, Humanoid, Anymal, FrankaCabinet, Quadcopter, ShadowHand, Ingenuity"},
        {"name": "--task_type", "type": str,
            "default": "Python", "help": "Choose Python or C++"},
        {"name": "--rl_device", "type": str, "default": "default",
            "help": "Choose CPU or GPU device for inferencing policy network"},
        {"name": "--cp_device", "type": str, "default": "default",
            "help": "Choose CPU or GPU device for training and inferencing collision predictor, available only for CP tasks"},
        {"name": "--logdir", "type": str, "default": os.path.join(DATA_DIR, 'isaacgym_exp')},
        {"name": "--experiment", "type": str, "default": "Base",
            "help": "Experiment name. If used with --metadata flag an additional information about physics engine, sim device, pipeline and domain randomization will be added to the name"},
        {"name": "--metadata", "action": "store_true", "default": False,
            "help": "Requires --experiment flag, adds physics engine, sim device, pipeline info and if domain randomization is used to the experiment name provided by user"},
        {"name": "--cfg_train", "type": str,
            "default": "Base"},
        {"name": "--cfg_env", "type": str, "default": "Base"},
        {"name": "--num_envs", "type": int, "default": 0,
            "help": "Number of environments to train - override config file"},
        {"name": "--num_envs_val", "type": int, "default": 0,
            "help": "Number of environments to validate - override config file"},
        {"name": "--num_objs", "type": int, "default": 0,
            "help": "Number of objects to train - override config file"},
        {"name": "--num_objs_val", "type": int, "default": 0,
            "help": "Number of objects to validate - override config file"},
        {"name": "--episode_length", "type": int, "default": 0,
            "help": "Episode length, by default is read from yaml config"},
        {"name": "--seed", "type": int, "help": "Random seed"},
        {"name": "--max_iterations", "type": int, "default": 0,
            "help": "Set a maximum number of training iterations"},
        {"name": "--steps_num", "type": int, "default": -1,
            "help": "Set number of simulation steps per 1 PPO iteration. Supported only by rl_games. If not -1 overrides the config settings."},
        {"name": "--minibatch_size", "type": int, "default": -1,
            "help": "Set batch size for PPO optimization step. Supported only by rl_games. If not -1 overrides the config settings."},
        {"name": "--randomize", "action": "store_true", "default": False,
            "help": "Apply physics domain randomization"},
        {"name": "--torch_deterministic", "action": "store_true", "default": False,
            "help": "Apply additional PyTorch settings for more deterministic behaviour"},
        {"name": "--algo", "type": str, "default": "happo",
            "help": "Choose an algorithm"},
        {"name": "--model_dir", "type": str, "default": "",
            "help": "Choose a model dir"},
        {"name": "--task_config", "type": str, "default": None,
            "help": "Whether to force config file for the task"},
        {"name": "--algo_config", "type": str, "default": None,
            "help": "Whether to force config file for the algorithm"},
        {"name": "--visualize_pc", "action": "store_true", "default": False,
            "help": "Open a window to show the point cloud of the first environment"},
        {"name": "--contact_buffer_size", "type": int, "default": None,
            "help": "Specify the size of contact buffer, default 512"},
        {"name": "--contrastive", "action": "store_true", "default": False,
            "help": "Whether do contrastive learning on pointnet"},
        {"name": "--contrastive_m", "type": int, "default": None,
            "help": "Specify contrastive momentum"},
        {"name": "--multitask", "action": "store_true", "default": False,
            "help": "Send taskid in observations"},
        {"name": "--no_mpr", "action": "store_true", "default": False,
            "help": "No maxpoint reward"},
        {"name": "--no_mpo", "action": "store_true", "default": False,
            "help": "no maxpoint observation"},
        {"name": "--cp_lr", "type": float, "default": -1,
            "help": "cp learning rate"},
        {"name": "--lr", "type": float, "default": -1,
            "help": "rl learning rate"},
        {"name": "--mpr", "type": float, "default": -1,
            "help": "maxpoint reward ratio"},
        {"name": "--use_image_obs", "type": bool, "default": False,
            "help": "Whether to use image observation"},
        {"name": "--isaacgym_task", "type": str, "default": "", "help": "Name of the isaacgym task to run"},
        {"name": "--eval_only", "type": bool, "default": False, "help": "Remove all training assets if True"},
    ]

    if benchmark:
        custom_parameters += [{"name": "--num_proc", "type": int, "default": 1, "help": "Number of child processes to launch"},
                              {"name": "--random_actions", "action": "store_true",
                                  "help": "Run benchmark with random actions instead of inferencing"},
                              {"name": "--bench_len", "type": int, "default": 10,
                                  "help": "Number of timing reports"},
                              {"name": "--bench_file", "action": "store", "help": "Filename to store benchmark results"}]


    # parse arguments
    if isaacgym_arg_str is None:
        args = gymutil.parse_arguments(
            description="RL Policy",
            custom_parameters=custom_parameters)
    else:
        from MARL_Module.envs.utils.parse_args_with_hydra import parse_arguments
        args = parse_arguments(
            description="RL Policy",
            custom_parameters=custom_parameters,
            isaacgym_arg_str=isaacgym_arg_str)

    # allignment with examples
    args.device_id = args.compute_device_id
    args.device = args.sim_device_type if args.use_gpu_pipeline else 'cpu'

    if args.test:
        args.play = args.test
        args.train = False
    elif args.play:
        args.train = False
    else:
        args.train = True

    # Override the task config with the isaacgym task config
    if len(args.isaacgym_task) > 0:
        from simple_bc.constants import ISAACGYM_TASK
        isaacgym_task = ISAACGYM_TASK[args.isaacgym_task]
        for key in ['expert_policy', 'task', 'task_config', 'algo']:
            args.__dict__[key] = isaacgym_task[key]


    logdir, cfg_train, cfg_env = retrieve_cfg(args, use_rlg_config)

    if use_rlg_config == False:
        if args.horovod:
            print("Distributed multi-gpu training with Horovod is not supported by rl-pytorch. Use rl_games for distributed training.")
        if args.steps_num != -1:
            print("Setting number of simulation steps per iteration from command line is not supported by rl-pytorch.")
        if args.minibatch_size != -1:
            print("Setting minibatch size from command line is not supported by rl-pytorch.")
        if args.checkpoint != "Base":
            raise ValueError("--checkpoint is not supported by rl-pytorch. Please use --resume <iteration number>")

    # use custom parameters if provided by user
    if args.logdir == "logs/":
        args.logdir = logdir

    if args.cfg_train == "Base":
        args.cfg_train = cfg_train

    if args.cfg_env == "Base":
        args.cfg_env = cfg_env

    if args.algo not in ["maddpg", "happo", "mappo", "mappo_pc_pure", "hatrpo","ippo","ppo","sac","td3","ddpg","trpo", "ppo_pc", "ppo_pc_pure", "sac_pc_pure"]:
        warn_algorithm_name()

    return args
