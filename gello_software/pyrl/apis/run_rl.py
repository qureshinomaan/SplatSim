import argparse
import glob
import os
import os.path as osp
import shutil
import time
import warnings
from copy import deepcopy
from pathlib import Path
import gym
import numpy as np

from pyrl.utils.meta import (
    Config,
    DictAction,
    add_dist_var,
    add_env_var,
    collect_env,
    colored_print,
    get_dist_info,
    get_logger,
    get_world_rank,
    is_debug_mode,
    set_cpu_random_seed,
    log_meta_info,
    get_total_memory,
)

warnings.simplefilter(action="ignore")


from pyrl.utils.data import is_not_null, is_null, num_to_str


def parse_args():
    parser = argparse.ArgumentParser(description="Unified API for Training and Evaluation")
    # Configurations
    parser.add_argument("config", help="Configuration file path")
    parser.add_argument(
        "--cfg-options",
        "--opt",
        nargs="+",
        action=DictAction,
        help="Override some settings in the configuration file. The key-value pair "
        "in xxx=yyy format will be merged into config file. If the value to "
        'be overridden is a list, it should be like key="[a,b]" or key=a,b '
        'It also allows nested list/tuple values, e.g. key="[(a,b),(c,d)]" '
        "Note that the quotation marks are necessary and that no white space "
        "is allowed.",
    )
    parser.add_argument("--debug", action="store_true", default=False)

    # Parameters for log dir
    parser.add_argument("--work-dir", help="The directory to save logs and models")
    parser.add_argument("--dev", action="store_true", default=False, help="Add timestamp to the name of work-dir")
    parser.add_argument("--with-agent-type", default=False, action="store_true", help="Add agent type to work-dir")
    parser.add_argument(
        "--agent-type-first",
        default=False,
        action="store_true",
        help="When work-dir is None, we will use agent_type/config_name or config_name/agent_type as work-dir",
    )
    parser.add_argument("--clean-up", help="Clean up the work-dir", action="store_true")

    # Evaluation mode
    parser.add_argument("--evaluation", "--eval", help="Evaluate a model, instead of training it", action="store_true")
    parser.add_argument("--reg-loss", help="Measure regression loss during evaluation", action="store_true")
    parser.add_argument("--test-name", help="Subdirectory name under work-dir to save the test result (if None, use {work-dir}/test)", default=None)

    # Resume checkpoint model
    parser.add_argument("--resume-from", default=None, nargs="+", help="A specific checkpoint file to resume from")
    parser.add_argument(
        "--auto-resume",
        help="Auto-resume the checkpoint under work-dir. If --resume-from is not specified, --auto-resume is set to True",
        action="store_true",
    )
    parser.add_argument("--resume-keys-map", default=None, nargs="+", action=DictAction, help="Specify how to change the model keys in checkpoints")

    # Specify GPU
    group_gpus = parser.add_mutually_exclusive_group()
    group_gpus.add_argument("--num-cpus", default=None, type=int, help="Number of gpus to use")
    group_gpus.add_argument("--num-gpus", default=None, type=int, help="Number of gpus to use")
    group_gpus.add_argument("--gpu-ids", default=None, type=int, nargs="+", help="ids of gpus to use")
    parser.add_argument(
        "--env-gpu-ids", default=None, type=int, nargs="+", help="ids of gpus for environment simulation; if not specified, this equals --gpu-ids"
    )

    # Torch and reproducibility settings
    parser.add_argument("--seed", type=int, default=None, help="Set torch and numpy random seed")
    parser.add_argument("--cudnn-benchmark", action="store_true", help="Whether to use benchmark mode in cudnn.")

    parser.add_argument("--deterministic", action="store_true", help="Whether to use deterministic mode for torch.")
    parser.add_argument(
        "--reproducible", action="store_true", help="Use deterministic mode also, the program will check the if the code is committed with git!"
    )

    # Distributed parameters
    # parser.add_argument('--launcher', choices=['none', 'pytorch', 'slurm', 'mpi'], default='none', help='job launcher')
    # parser.add_argument('--local-rank', type=int, default=0)
    args = parser.parse_args()

    # Merge cfg with args.cfg_options
    cfg = Config.fromfile(args.config)
    if args.cfg_options is not None:
        for key, value in args.cfg_options.items():
            try:
                value = eval(value)
                args.cfg_options[key] = value
            except:
                pass
        cfg.merge_from_dict(args.cfg_options)

    args.with_agent_type = args.with_agent_type or args.agent_type_first
    for key in ["work_dir", "env_cfg", "resume_from", "eval_cfg", "replay_cfg", "expert_replay_cfg", "recent_traj_replay_cfg", "rollout_cfg"]:
        cfg[key] = cfg.get(key, None)
    if args.debug:
        os.environ["PYRL_DEBUG"] = "True"
    elif "PYRL_DEBUG" not in os.environ:
        os.environ["PYRL_DEBUG"] = "False"
    if args.seed is None:
        args.seed = np.random.randint(2**32 - int(1e8))
        args.deterministic = args.deterministic
    else:
        args.deterministic = True

    if args.reproducible:
        args.deterministic = True

    if args.evaluation:
        from pyrl.methods.builder import SL

        args.reg_loss = args.reg_loss or cfg.agent_cfg.type in SL
    args.mode = "eval" if args.evaluation else "train"

    if args.num_cpus:
        pass
    return args, cfg


def build_work_dir():
    if is_null(args.work_dir):
        root_dir = "./work_dirs"
        env_name = cfg.env_cfg.get("env_name", None) if is_not_null(cfg.env_cfg) else None
        config_name = osp.splitext(osp.basename(args.config))[0]
        folder_name = env_name if is_not_null(env_name) else config_name
        if args.with_agent_type:
            if args.agent_type_first:
                args.work_dir = osp.join(root_dir, agent_type, folder_name)
            else:
                args.work_dir = osp.join(root_dir, folder_name, agent_type)
        else:
            args.work_dir = osp.join(root_dir, folder_name)
    elif args.with_agent_type:
        if args.agent_type_first:
            colored_print("When you specify the work dir path, the agent type cannot be at the beginning of the path!", level="warning")
        args.work_dir = osp.join(args.work_dir, agent_type)

    if args.dev:
        splits = list(osp.split(args.work_dir))
        splits[1] += "-dev"
        args.work_dir = osp.join(*splits)
        args.work_dir = osp.join(args.work_dir, args.timestamp)

    if args.clean_up:
        if args.evaluation or args.auto_resume or (is_not_null(args.resume_from) and os.path.commonprefix(args.resume_from) == args.work_dir):
            colored_print(
                "We will ignore the clean-up flag, since we are either in the evaluation mode or resuming from the directory!", level="warning"
            )
        else:
            colored_print(
                f"We will delete the work_dir {args.work_dir}!", level="error"
            )
            shutil.rmtree(args.work_dir, ignore_errors=True)
    os.makedirs(osp.abspath(args.work_dir), exist_ok=True)


def find_checkpoint():
    logger = get_logger()
    if is_not_null(args.resume_from):
        if is_not_null(cfg.resume_from):
            colored_print(f"The resumed checkpoint from the config file is overwritten by {args.resume_from}!", level="warning")
        cfg.resume_from = args.resume_from

    if args.auto_resume or (args.evaluation and is_null(cfg.resume_from)):
        logger.info(f"Search model under {args.work_dir}.")
        model_names = list(glob.glob(osp.join(args.work_dir, "models", "*.ckpt")))
        latest_index = -1
        latest_name = None
        for model_i in model_names:
            index_str = osp.basename(model_i).split(".")[0].split("_")[1]
            if index_str == "final":
                continue
            index = eval(index_str)
            if index > latest_index:
                latest_index = index
                latest_name = model_i

        if is_null(latest_name):
            colored_print(f"Find no checkpoints under {args.work_dir}!", level="warning")
        else:
            cfg.resume_from = latest_name
            cfg.train_cfg["resume_steps"] = latest_index
    if is_not_null(cfg.resume_from):
        if isinstance(cfg.resume_from, str):
            cfg.resume_from = [
                cfg.resume_from,
            ]
        logger.info(f"Get {len(cfg.resume_from)} checkpoint {cfg.resume_from}.")
        logger.info(f"Check checkpoint {cfg.resume_from}!")

        for file in cfg.resume_from:
            if not (osp.exists(file) and osp.isfile(file)):
                logger.error(f"Checkpoint file {file} does not exist!")
                exit(-1)


def get_python_env_info():
    if is_not_null(args.num_gpus) and is_not_null(args.gpu_ids):
        colored_print("Please use either 'num-gpus' or 'gpu-ids'!", level="error")
        exit(0)

    if is_not_null(args.num_gpus):
        args.gpu_ids = list(range(args.num_gpus))
        args.num_gpus = None
    if args.gpu_ids is None:
        args.gpu_ids = []

    if args.evaluation and len(args.gpu_ids) > 1:
        colored_print(f"Multiple GPU evaluation is not supported; we will use the first GPU to do evaluation!", level="warning")
        args.gpu_ids = args.gpu_ids[:1]


def set_env_paras():
    from pyrl.methods.builder import SL

    agent_type = cfg.agent_cfg.type
    if agent_type in ["CEM", "PETS"]:
        cfg.rollout_cfg.type = "Rollout"
        cfg.eval_cfg.type = "Evaluation"
        cfg.eval_cfg.num_procs = 1
    elif agent_type in SL:
        # For supervised learning, we do not need to build rollout and evaluation environment
        cfg.env_cfg = None
        cfg.rollout_cfg = None
        cfg.eval_cfg = None


def init_torch(args):
    import torch

    torch.utils.backcompat.broadcast_warning.enabled = True
    torch.utils.backcompat.keepdim_warning.enabled = True
    torch.backends.cudnn.benchmark = args.cudnn_benchmark

    rank = get_world_rank()
    if args.gpu_ids is not None and len(args.gpu_ids) > 0:
        torch.cuda.set_device(args.gpu_ids[rank])
        torch.set_num_threads(1)

    if is_debug_mode():
        torch.autograd.set_detect_anomaly(True)

    torch.manual_seed(args.seed + rank)
    if torch.cuda.is_available():
        torch.cuda.manual_seed(args.seed + rank)
        torch.cuda.manual_seed_all(args.seed + rank)

    if args.deterministic:
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.benchmark = False

    torch.set_printoptions(precision=10, sci_mode=False)


def main_rl(rollout, evaluator, replay, args, cfg, expert_replay=None, recent_traj_replay=None):
    import torch
    import torch.distributed as dist
    import torch.nn as nn
    from pyrl.apis.train_rl import train_rl
    from pyrl.env import save_eval_statistics
    from pyrl.methods.builder import build_agent
    from pyrl.utils.data.converter import dict_to_str
    from pyrl.utils.torch import BaseAgent, load_checkpoint, save_checkpoint

    logger = get_logger()
    logger.info("Initialize torch!")
    init_torch(args)

    logger.info("Finish Initialize torch!")
    world_rank, world_size = get_dist_info()

    if is_not_null(cfg.agent_cfg.get("batch_size", None)) and isinstance(cfg.agent_cfg.batch_size, (list, tuple)):
        assert len(cfg.agent_cfg.batch_size) == len(args.gpu_ids)
        cfg.agent_cfg.batch_size = cfg.agent_cfg.batch_size[world_rank]
        logger.info(f"Set batch size to {cfg.agent_cfg.batch_size}!")

    logger.info("Build agent!")
    agent = build_agent(cfg.agent_cfg)

    assert agent is not None, f"Agent type {cfg.agent_cfg.type} is not valid!"

    logger.info(agent)
    logger.info(
        f'Num of parameters: {num_to_str(agent.num_trainable_parameters, unit="M")}, Model Size: {num_to_str(agent.size_trainable_parameters, unit="M")}'
    )
    device = "cpu" if len(args.gpu_ids) == 0 else "cuda"
    agent = agent.to(device)
    assert isinstance(agent, BaseAgent), "The agent object should be an instance of BaseAgent!"

    if is_not_null(cfg.resume_from):
        logger.info("Resume agent with checkpoint!")
        for file in cfg.resume_from:
            load_checkpoint(agent, file, device, keys_map=args.resume_keys_map, logger=logger)

    if len(args.gpu_ids) > 1:
        logger.info("Setting DDP!")
        assert not args.evaluation, "We do not support multi-gpu evaluation!"
        os.environ["MASTER_ADDR"] = "localhost"
        os.environ["MASTER_PORT"] = "12355"
        dist.init_process_group("nccl", rank=world_rank, world_size=world_size)
        # from pyrl.utils.torch import ExtendedDDP
        agent = nn.SyncBatchNorm.convert_sync_batchnorm(agent)
        try:
            from torchsparse.nn.modules import SyncBatchNorm as SpSyncBatchNorm

            agent = SpSyncBatchNorm.convert_sync_batchnorm(agent)
        except:
            pass
        agent.to_ddp(device_ids=["cuda"])

    logger.info(f"Work directory of this run {args.work_dir}")
    if len(args.gpu_ids) > 0:
        logger.info(f"Train over GPU {args.gpu_ids}!")
    else:
        logger.info(f"Train over CPU!")

    if not args.evaluation:
        train_rl(
            agent,
            rollout,
            evaluator,
            replay,
            work_dir=args.work_dir,
            eval_cfg=cfg.eval_cfg,
            expert_replay=expert_replay,
            recent_traj_replay=recent_traj_replay,
            clean_up=args.clean_up,
            timestamp=args.timestamp,  # if args.dev else None,
            **cfg.train_cfg,
        )
    else:
        agent.eval()
        agent.set_mode("test")

        if is_not_null(replay) and args.reg_loss:
            loss_dict = agent.compute_test_loss(replay)
            logger.info(dict_to_str(loss_dict))
        if is_not_null(evaluator):
            # For RL
            lens, rewards, finishes = evaluator.run(agent, work_dir=work_dir, **cfg.eval_cfg)
            save_eval_statistics(work_dir, lens, rewards, finishes)
        agent.train()
        agent.set_mode("train")

    if len(args.gpu_ids) > 1:
        dist.destroy_process_group()


def run_one_process(rank, world_size, args, cfg):
    import numpy as np

    args.seed += rank

    add_dist_var(rank, world_size)
    set_cpu_random_seed(args.seed)
    np.set_printoptions(precision=5, suppress=True)

    if is_not_null(cfg.env_cfg) and len(args.gpu_ids) > 0:
        if args.env_gpu_ids is not None:
            assert len(args.env_gpu_ids) == len(args.gpu_ids), "Number of simulation gpus should be the same as the number of training gpus!"
        else:
            args.env_gpu_ids = args.gpu_ids
        cfg.env_cfg.device = f"cuda:{args.env_gpu_ids[rank]}"

    work_dir = args.work_dir
    logger_file = osp.join(work_dir, f"{args.timestamp}-{args.name_suffix}.log")
    logger = get_logger(name=None, log_file=logger_file, log_level=cfg.get("log_level", "INFO"))

    if is_debug_mode():
        dash_line = "-" * 60 + "\n"
        logger.info("Environment info:\n" + dash_line + args.env_info + "\n" + dash_line)

    logger.info(f"Config:\n{cfg.pretty_text}")
    logger.info(f"Set random seed to {args.seed}")

    # Create replay buffer for RL
    if is_not_null(cfg.replay_cfg) and (not args.evaluation or (args.reg_loss and cfg.replay_cfg.get("buffer_filenames", None) is not None)):
        logger.info(f"Build replay buffer!")
        from pyrl.env import build_replay

        replay = build_replay(cfg.replay_cfg)
        expert_replay, recent_traj_replay = None, None
        if is_not_null(cfg.expert_replay_cfg):
            assert cfg.expert_replay_cfg.buffer_filenames is not None
            expert_replay = build_replay(cfg.expert_replay_cfg)
        if is_not_null(cfg.recent_traj_replay_cfg):
            recent_traj_replay = build_replay(cfg.recent_traj_replay_cfg)
    else:
        replay = None
        expert_replay = None
        recent_traj_replay = None

    # Create rollout module for online methods
    if not args.evaluation and is_not_null(cfg.rollout_cfg):
        from pyrl.env import build_rollout

        logger.info(f"Build rollout! Total memory before build rollout: {get_total_memory()}!")
        rollout_cfg = cfg.rollout_cfg
        rollout_cfg["env_cfg"] = deepcopy(cfg.env_cfg)
        rollout = build_rollout(rollout_cfg)
    else:
        rollout = None

    # Build evaluation module
    if is_not_null(cfg.eval_cfg) and rank == 0:
        # Only the first process will do evaluation
        from pyrl.env import build_evaluation

        logger.info(f"Build evaluation!")
        eval_cfg = cfg.eval_cfg
        # Evaluation environment setup can be different from the training setup. (Like early-stop or object sets)
        if eval_cfg.get("env_cfg", None) is None:
            eval_cfg["env_cfg"] = deepcopy(cfg.env_cfg)
        else:
            tmp = eval_cfg["env_cfg"]
            eval_cfg["env_cfg"] = deepcopy(cfg.env_cfg)
            eval_cfg["env_cfg"].update(tmp)
        get_logger().info(f"Building evaluation: eval_cfg: {eval_cfg}")
        evaluator = build_evaluation(eval_cfg)
    else:
        evaluator = None

    # Get environments information for agents
    obs_shape, action_shape = None, None
    if is_not_null(cfg.env_cfg):
        # For RL which needs environments
        logger.info(f"Get obs shape!")
        from pyrl.env import get_env_info

        if rollout is not None:
            env_params = get_env_info(cfg.env_cfg, rollout.vec_env)
        elif hasattr(evaluator, "vec_env"):
            env_params = get_env_info(cfg.env_cfg, evaluator.vec_env)
        else:
            env_params = get_env_info(cfg.env_cfg)
        cfg.agent_cfg["env_params"] = env_params
        obs_shape = env_params["obs_shape"]
        action_shape = env_params["action_shape"]
        logger.info(f'State shape:{env_params["obs_shape"]}, action shape:{env_params["action_shape"]}')
        logger.info(env_params["message"])
    elif is_not_null(replay):
        obs_shape = None
        for obs_key in ["inputs", "obs"]:
            if obs_key in replay.memory:
                obs_shape = replay.memory.slice(0).shape[obs_key]
                break

    if is_not_null(obs_shape) or is_not_null(action_shape):
        from pyrl.networks.utils import get_kwargs_from_shape, replace_placeholder_with_args

        replaceable_kwargs = get_kwargs_from_shape(obs_shape, action_shape)
        cfg = replace_placeholder_with_args(cfg, **replaceable_kwargs)
    logger.info(f"Final agent config:\n{cfg.agent_cfg}")

    # Output version of important packages
    log_meta_info(logger)

    main_rl(rollout, evaluator, replay, args, cfg, expert_replay=expert_replay, recent_traj_replay=recent_traj_replay)

    if rank == 0:
        env_info_dict = collect_env()
        args.env_info = "\n".join([f"{k}: {v}" for k, v in env_info_dict.items()])

    if is_not_null(evaluator):
        evaluator.close()
        logger.info("Close evaluator object")
    if is_not_null(rollout):
        rollout.close()
        logger.info("Close rollout object")
    if is_not_null(replay):
        replay.close()
        logger.info("Delete replay buffer")


def main():
    if len(args.gpu_ids) > 1:
        import torch.multiprocessing as mp

        world_size = len(args.gpu_ids)
        mp.spawn(run_one_process, args=(world_size, args, cfg), nprocs=world_size, join=True)
    else:
        run_one_process(0, 1, args, cfg)


if __name__ == "__main__":
    # Remove mujoco_py lock
    mjpy_lock = Path(gym.__file__).parent.parent / "mujoco_py/generated/mujocopy-buildlock.lock"
    if mjpy_lock.exists():
        os.remove(str(mjpy_lock))

    add_env_var()

    args, cfg = parse_args()
    args.timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    agent_type = cfg.agent_cfg.type

    if args.reproducible:
        from pyrl.utils.meta import check_reproducibility

        check_reproducibility()


    build_work_dir()
    find_checkpoint()
    get_python_env_info()

    work_dir = args.work_dir
    if args.evaluation:
        test_name = args.test_name if args.test_name is not None else "test"
        work_dir = osp.join(work_dir, test_name)
        # Always clean up for evaluation
        shutil.rmtree(work_dir, ignore_errors=True)
        os.makedirs(work_dir, exist_ok=True)
    args.work_dir = work_dir

    logger_name = cfg.env_cfg.env_name if is_not_null(cfg.env_cfg) else cfg.agent_cfg.type
    args.name_suffix = f"{args.mode}"
    if args.test_name is not None:
        args.name_suffix += f"-{args.test_name}"
    os.environ["PYRL_LOGGER_NAME"] = f"{logger_name}-{args.name_suffix}"
    cfg.dump(osp.join(work_dir, f"{args.timestamp}-{args.name_suffix}.py"))

    main()
