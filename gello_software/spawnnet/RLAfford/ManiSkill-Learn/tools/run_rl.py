import argparse
import glob
import os
import os.path as osp
import shutil
import time
from copy import deepcopy

os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"

from mani_skill_learn.env import save_eval_statistics
from mani_skill_learn.utils.meta import Config, DictAction, set_random_seed, collect_env, get_logger


def init_torch(args):
    import torch
    torch.utils.backcompat.broadcast_warning.enabled = True
    torch.utils.backcompat.keepdim_warning.enabled = True
    torch.backends.cudnn.benchmark = args.cudnn_benchmark

    if args.gpu_ids is not None:
        torch.cuda.set_device(args.gpu_ids[args.local_rank])


def parse_args():
    parser = argparse.ArgumentParser(description='Run RL training code')
    # Configurations
    parser.add_argument('config', help='train config file path')
    parser.add_argument('--cfg-options', nargs='+', action=DictAction,
                        help='override some settings in the used config, the key-value pair '
                             'in xxx=yyy format will be merged into config file. If the value to '
                             'be overwritten is a list, it should be like key="[a,b]" or key=a,b '
                             'It also allows nested list/tuple values, e.g. key="[(a,b),(c,d)]" '
                             'Note that the quotation marks are necessary and that no white space '
                             'is allowed.')

    # Parameters for log dir
    parser.add_argument('--work-dir', help='the dir to save logs and models')
    parser.add_argument('--dev', action='store_true', default=False)

    dir_agent = parser.add_mutually_exclusive_group()
    dir_agent.add_argument('--no-agent-type', default=False, action='store_true', help='no agent type')
    dir_agent.add_argument('--agent-type-first', default=False, action='store_true',
                           help='when work-dir is None, we will use agent_type/config_name or config_name/agent_type')

    parser.add_argument('--clean-up', help='Clean up the work_dir', action='store_true')

    # If we use evaluation mode
    parser.add_argument('--evaluation', help='Use evaluation mode', action='store_true')
    parser.add_argument('--test-name', help='The name of the folder to save the test result', default=None)

    # If we resume checkpoint model
    parser.add_argument('--resume-from', default=None, help='the checkpoint file to resume from')
    parser.add_argument('--auto-resume', help='Auto-resume the checkpoint under work dir, '
                                              'the default value is true when in evaluation mode', action='store_true')

    # Specify GPU
    group_gpus = parser.add_mutually_exclusive_group()
    group_gpus.add_argument('--num-gpus', default=None, type=int, help='number of gpus to use')
    group_gpus.add_argument('--gpu-ids', default=None, type=int, nargs='+', help='ids of gpus to use')

    # Torch and reproducibility settings
    parser.add_argument('--seed', type=int, default=None, help='random seed')
    parser.add_argument('--cudnn_benchmark', action='store_true', help='whether to use benchmark mode in cudnn.')

    # Distributed parameters
    parser.add_argument('--launcher', choices=['none', 'pytorch', 'slurm', 'mpi'], default='none', help='job launcher')
    parser.add_argument('--local_rank', type=int, default=0)
    args = parser.parse_args()
    return args


def main_mfrl_brl(cfg, args, rollout, evaluator, logger):
    from mani_skill_learn.methods.builder import build_mfrl, build_brl, MFRL, BRL
    from mani_skill_learn.apis.train_rl import train_rl
    from mani_skill_learn.env import build_replay
    from mani_skill_learn.utils.torch import load_checkpoint
    import torch.distributed as dist, torch

    init_torch(args)

    if cfg.agent.type in MFRL:
        agent = build_mfrl(cfg.agent)
    elif cfg.agent.type in BRL:
        agent = build_brl(cfg.agent)
    else:
        raise NotImplementedError("")

    if cfg.get('resume_from', None) is not None:
        load_checkpoint(agent, cfg.resume_from, map_location='cpu')

    if args.gpu_ids is not None and len(args.gpu_ids) > 0:
        agent.to('cuda')

    if (cfg.get('eval_cfg', None) is not None and (cfg.eval_cfg.get('num_procs', 1) > 1 and
                                                  not cfg.eval_cfg.get('synchronize', True)) or
        (cfg.get('rollout_cfg', None) is not None and cfg.rollout_cfg.get('num_procs', 1) > 1 and
                                                  not cfg.eval_cfg.get('synchronize', True))):
        agent.share_memory()

    if not (dist.is_available() and dist.is_initialized()):
        logger.info("We do not use distributed training, but we support data parallel in torch")
    assert args.local_rank == 0
    if len(args.gpu_ids) > 1:
        logger.warning("Use Data parallel to train model! It may slow down the speed when the model is small.")
        agent.to_data_parallel(device_ids=args.gpu_ids, output_device=torch.cuda.current_device())

    if not args.evaluation:
        replay = build_replay(cfg.replay_cfg)
        train_rl(agent, rollout, evaluator, cfg.env_cfg, replay, work_dir=cfg.work_dir, eval_cfg=cfg.eval_cfg,
                 **cfg.train_mfrl_cfg)
    else:
        test_name = args.test_name if args.test_name is not None else 'test'
        eval_dir = osp.join(cfg.work_dir, test_name)
        shutil.rmtree(eval_dir, ignore_errors=True)
        agent.eval()
        lens, rewards, finishes = evaluator.run(agent, work_dir=eval_dir, **cfg.eval_cfg)
        save_eval_statistics(eval_dir, lens, rewards, finishes, logger)
        agent.train()


def main():
    timestamp = time.strftime('%Y%m%d_%H%M%S', time.localtime())

    # Process args and merge cfg with args
    args = parse_args()
    cfg = Config.fromfile(args.config)
    if args.cfg_options is not None:
        cfg.merge_from_dict(args.cfg_options)
    if args.work_dir is not None:
        cfg.work_dir = args.work_dir

    if args.resume_from is not None:
        cfg.resume_from = args.resume_from
        args.auto_resume = False

    if not args.evaluation and cfg.get('rollout_cfg', None) is not None:
        from mani_skill_learn.env import build_rollout
        # Create environment rollout before load torch and other library to
        # reduce memory cost for multiprocess data collection
        rollout_cfg = cfg.rollout_cfg
        rollout_cfg['env_cfg'] = deepcopy(cfg.env_cfg)
        rollout = build_rollout(rollout_cfg)
    else:
        rollout = None

    if cfg.get('eval_cfg', None) is not None:
        from mani_skill_learn.env import build_evaluation
        eval_cfg = cfg.eval_cfg
        eval_cfg['env_cfg'] = deepcopy(cfg.env_cfg)
        evaluator = build_evaluation(eval_cfg)
    else:
        evaluator = None

    # import modules from string list.
    if cfg.get('custom_imports', None):
        from mani_skill_learn.utils import import_modules_from_strings
        import_modules_from_strings(**cfg['custom_imports'])

    # work_dir is determined in this priority: CLI > segment in file > filename
    if cfg.get('work_dir', None) is not None:
        if not args.no_agent_type:
            cfg.work_dir = osp.join(cfg.work_dir, cfg.agent.type)
    else:
        # use config filename as default work_dir if cfg.work_dir is None
        root_dir = './work_dirs'
        folder_name = cfg.env_cfg.get('env_name', None)
        if folder_name is None:
            folder_name = osp.splitext(osp.basename(args.config))[0]
        if args.agent_type_first:
            if args.no_agent_type:
                print('Ignore no_agent_type when you require agent_type_first!')
            cfg.work_dir = osp.join(root_dir, cfg.agent.type, folder_name)
        else:
            cfg.work_dir = osp.join(root_dir, folder_name)
            if not args.no_agent_type:
                cfg.work_dir = osp.join(cfg.work_dir, cfg.agent.type)
    if args.dev:
        cfg.work_dir = osp.join(cfg.work_dir, timestamp)

    if args.clean_up:
        if args.evaluation or args.auto_resume or \
                (args.resume_from is not None and os.path.commonprefix(args.resume_from) == cfg.work_dir):
            print('The system will ignore the clean-up flag, '
                  'when we are in evaluation mode or resume from the directory!')
        else:
            shutil.rmtree(cfg.work_dir, ignore_errors=True)
    # create work_dir
    os.makedirs(osp.abspath(cfg.work_dir), exist_ok=True)
    # dump config
    if not args.evaluation:
        cfg.dump(osp.join(cfg.work_dir, osp.basename(args.config)))
        # init the logger before other steps
        logger = get_logger(name=cfg.env_cfg.env_name, log_file=osp.join(cfg.work_dir, f'{timestamp}.log'),
                            log_level=cfg.log_level)
    else:
        # Evaluation mode, we do not use logging
        logger = get_logger(name=cfg.env_cfg.env_name, log_level=cfg.log_level)

    env_info_dict = collect_env()
    env_info = '\n'.join([f'{k}: {v}' for k, v in env_info_dict.items()])
    dash_line = '-' * 60 + '\n'
    logger.info('Environment info:\n' + dash_line + env_info + '\n' + dash_line)

    # log some basic info
    logger.info(f'Config:\n{cfg.pretty_text}')

    # Seed random seed
    logger.info(f'Set random seed to {args.seed}')
    set_random_seed(args.seed)

    num_gpus = env_info_dict['Num of GPUs']
    if args.num_gpus is not None and args.gpu_ids is not None:
        logger.error("Please use only one of 'num-gpus' and 'gpu-ids'")
        exit(0)
    elif args.num_gpus is None and args.gpu_ids is None:
        if env_info_dict['num_gpus'] > 0:
            logger.warning(f'We will use cpu to do training, although we have {num_gpus} gpus!')
    if args.gpu_ids is not None:
        args.gpu_ids = args.gpu_ids
    elif args.num_gpus is not None:
        assert args.num_gpus <= num_gpus
        args.gpu_ids = list(range(args.num_gpus))
    else:
        args.gpu_ids = None

    if args.auto_resume or (args.evaluation and cfg.get('resume_from', None) is None):
        logger.info(f'Search model in {cfg.work_dir}')
        model_names = list(glob.glob(osp.join(cfg.work_dir, 'models', '*.ckpt')))
        latest_index = -1
        latest_name = None
        for model_i in model_names:
            index = eval(osp.basename(model_i).split('.')[0].split('_')[1])
            if index > latest_index:
                latest_index = index
                latest_name = model_i
        if latest_name is not None:
            logger.info(f'Get model {latest_name}')
            cfg.resume_from = latest_name

    if cfg.get('env_cfg', None) is not None:
        from mani_skill_learn.env import get_env_info
        obs_shape, action_shape, action_space = get_env_info(cfg.env_cfg)
        cfg.agent['obs_shape'] = obs_shape
        cfg.agent['action_shape'] = action_shape
        cfg.agent['action_space'] = action_space
        logger.info(f'State shape:{obs_shape}, action shape:{action_space}')

    from mani_skill_learn.methods.builder import MFRL, BRL
    if cfg.agent.type in MFRL or cfg.agent.type in BRL:
        main_mfrl_brl(cfg, args, rollout, evaluator, logger)
    else:
        logger.error(f'No such agent type {cfg.agent.type}')

    if evaluator is not None:
        evaluator.close()
    if rollout is not None:
        rollout.close()


if __name__ == '__main__':
    main()
