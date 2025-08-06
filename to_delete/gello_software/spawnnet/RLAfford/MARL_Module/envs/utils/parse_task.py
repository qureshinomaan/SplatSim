# Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.


from tasks.franka_cabinet import OneFrankaCabinet
from tasks.franka_cabinet_PC_partial import OneFrankaCabinetPCPartial
from tasks.franka_cabinet_PC_partial_cp_map import OneFrankaCabinetPCPartialCPMap
from tasks.franka_cabinet_PC_partial_cp_state import OneFrankaCabinetPCPartialCPState
from tasks.franka_cabinet_PC_where2act import OneFrankaCabinetPCWhere2act
from tasks.franka_grasp import OneFrankaGrasp
from tasks.franka_pap import PAPRaw
# from tasks.franka_push_stapler import PushStapler
# from tasks.franka_open_pot import OpenPot

from tasks.franka_pap_partial import PAPPartial
# from tasks.franka_pap_a2o import PAPA2O
from tasks.franka_grasp_PC_partial import OneFrankaGraspPCPartial
from tasks.franka_chair import TwoFrankaChair
from tasks.franka_chair_PC_partial import TwoFrankaChairPCPartial
from tasks.franka_chair_PC_partial_cp_map import TwoFrankaChairPCPartialCPMap
from tasks.franka_chair_PC_partial_multi_agent import TwoFrankaChairPCPartialMultiAgent
from tasks.franka_cabinet_real_world import OneFrankaCabinetRealWorld
from tasks.franka_chair_realworld import TwoFrankaChairRealWorld
# from tasks.franka_cabinet_PC_partial_intime_map import OneFrankaCabinetPCPartialIntimeMap
# from tasks.franka_cabinet_PC_partial_pure_map import OneFrankaCabinetPCPartialPureMap
# from tasks.franka_chair import OneFrankaChair
# from tasks.franka_chair_PC_partial import OneFrankaChairPCPartial
from tasks.hand_base.vec_task import VecTaskCPU, VecTaskGPU, VecTaskPython, VecTaskPythonArm, VecTaskPythonArmPCPure
from tasks.hand_base.multi_vec_task import MyMultiTask

from MARL_Module.envs.utils.config import warn_task_name

import json

def parse_task(args, cfg, cfg_train, sim_params, agent_index, log_dir):
    if args.use_image_obs:
        print('Setting enableCameraSensors to True for image observations')
        cfg['env']['enableCameraSensors'] = True

    # create native task and pass custom config
    device_id = args.device_id
    rl_device = args.rl_device

    cfg["seed"] = cfg_train.get("seed", -1)
    cfg_task = cfg["env"]
    cfg_task["seed"] = cfg["seed"]


    log_dir = log_dir + "_seed{}".format(cfg_task["seed"])

    if args.task_type == "C++":
        if args.device == "cpu":
            print("C++ CPU")
            task = rlgpu.create_task_cpu(args.task, json.dumps(cfg_task))
            if not task:
                warn_task_name()
            if args.headless:
                task.init(device_id, -1, args.physics_engine, sim_params)
            else:
                task.init(device_id, device_id, args.physics_engine, sim_params)
            env = VecTaskCPU(task, rl_device, False, cfg_train.get("clip_observations", 5.0), cfg_train.get("clip_actions", 1.0))
        else:
            print("C++ GPU")

            task = rlgpu.create_task_gpu(args.task, json.dumps(cfg_task))
            if not task:
                warn_task_name()
            if args.headless:
                task.init(device_id, -1, args.physics_engine, sim_params)
            else:
                task.init(device_id, device_id, args.physics_engine, sim_params)
            env = VecTaskGPU(task, rl_device, cfg_train.get("clip_observations", 5.0), cfg_train.get("clip_actions", 1.0))

    elif args.task_type == "Python":
        print("Python")

        try:
            task = eval(args.task)(
                cfg=cfg,
                sim_params=sim_params,
                physics_engine=args.physics_engine,
                device_type=args.device,
                device_id=device_id,
                headless=args.headless,
                is_multi_agent=False,
                log_dir=log_dir)
            print(task)
        except NameError as e:
            print(e)
            warn_task_name()

        if args.task == "OneFrankaCabinet" :
            env = VecTaskPythonArm(task, rl_device)
        elif args.task == "OneFrankaCabinetPCPure" :
            env = VecTaskPythonArm(task, rl_device)
        elif args.task == "OneFrankaCabinetPCPartial" :
            env = VecTaskPythonArmPCPure(task, rl_device)
        elif args.task == "OneFrankaCabinetPCPartialPureMap" :
            env = VecTaskPythonArmPCPure(task, rl_device)
        elif args.task == "OneFrankaCabinetPCPartialCPMap" :
            env = VecTaskPythonArmPCPure(task, rl_device)
        elif args.task == "OneFrankaCabinetPCWhere2act" :
            env = VecTaskPythonArmPCPure(task, rl_device)
        elif args.task == "OneFrankaCabinetPCPartialIntimeMap" :
            env = VecTaskPythonArmPCPure(task, rl_device)
        elif args.task == "OneFrankaChair" :
            env = VecTaskPythonArmPCPure(task, rl_device)
        elif args.task == "OneFrankaChairPCPartial" :
            env = VecTaskPythonArmPCPure(task, rl_device)
        elif args.task == "OneFrankaGraspPCPartial" :
            env = VecTaskPythonArmPCPure(task, rl_device)
        elif args.task == "OneFrankaCabinetPCPartialCPState" :
            env = VecTaskPythonArm(task, rl_device)
        elif args.task == "OneFrankaCabinetPCPartialCPState" :
            env = VecTaskPythonArm(task, rl_device)
        elif args.task == "OneFrankaCabinetRealWorld" :
            env = VecTaskPythonArm(task, rl_device)
        elif args.task == "PAPRaw" :
            env = VecTaskPythonArm(task, rl_device)
        elif args.task == "PushStapler" :
            env = VecTaskPythonArm(task, rl_device)
        elif args.task == "OpenPot" :
            env = VecTaskPythonArm(task, rl_device)
        elif args.task == "PAPPartial" :
            env = VecTaskPythonArmPCPure(task, rl_device)
        elif args.task == "PAPA2O" :
            env = VecTaskPythonArmPCPure(task, rl_device)
        elif args.task == "TwoFrankaChair" :
            env = VecTaskPythonArm(task, rl_device)
        elif args.task == "TwoFrankaChairPCPartial" :
            env = VecTaskPythonArmPCPure(task, rl_device)
        elif args.task == "TwoFrankaChairPCPartialCPMap" :
            env = VecTaskPythonArmPCPure(task, rl_device)
        elif args.task == 'TwoFrankaChairRealWorld' :
            env = VecTaskPythonArm(task, rl_device)
        else:
            env = VecTaskPython(task, rl_device)

        if args.use_image_obs:
            from MARL_Module.envs.tasks.image_wrapper import ImageWrapper
            env = ImageWrapper(env)

    elif args.task_type == "MultiAgent":
        print("MultiAgent")

        try:
            task = eval(args.task)(
                cfg=cfg,
                sim_params=sim_params,
                physics_engine=args.physics_engine,
                device_type=args.device,
                device_id=device_id,
                headless=args.headless,
                agent_index=agent_index,
                is_multi_agent=True)
        except NameError as e:
            print(e)
            warn_task_name()
        env = MyMultiTask(task, rl_device)

    return task, env
