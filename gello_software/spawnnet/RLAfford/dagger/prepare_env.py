import os
from natsort import natsorted

def filter_assets(cfg, chunk_id, chunk_size, mode):
    assert mode in ["train", "val"]
    if mode == 'train':
        num_env_key = 'numTrain'
        num_assets_key = 'cabinetAssetNumTrain'
        assets_key = 'trainAssets'
        cfg['env']['numVal'] = 0
        cfg['env']['asset']['cabinetAssetNumVal'] = 0
    else:
        num_env_key = 'numVal'
        num_assets_key = 'cabinetAssetNumVal'
        assets_key = 'testAssets'
        cfg['env']['numTrain'] = 0
        cfg['env']['asset']['cabinetAssetNumTrain'] = 0
    total_assets = cfg["env"]["asset"][num_assets_key]
    
    if mode == 'train' and os.environ.get('TRAIN_ASSET_SPLIT') is not None:
        asset_sizes = os.environ.get('TRAIN_ASSET_SPLIT')
        asset_sizes = [int(x) for x in asset_sizes.split(',')]

        assert sum(asset_sizes) == total_assets, "Must use exactly all assets. Usage: {}, Expected Usage: {}".format(sum(asset_sizes), total_assets)

        num_assets = asset_sizes[chunk_id]
        asset_start_idx = sum(asset_sizes[:chunk_id])
    else:
        if chunk_id == chunk_size - 1:
            num_assets = total_assets - (chunk_size - 1) * (total_assets // chunk_size)
        else:
            num_assets = total_assets // chunk_size
        
        asset_start_idx = chunk_id * (total_assets // chunk_size)
    
    cfg['env'][num_env_key] = num_assets
    cfg['env']['asset'][num_assets_key] = num_assets
          
    asset_infos = cfg["env"]["asset"][assets_key]
    asset_names = natsorted(asset_infos.keys())
    cfg["env"]["asset"][assets_key] = {asset_names[i]: asset_infos[asset_names[i]] for i in
                                       range(asset_start_idx, asset_start_idx + num_assets)}
    assets_chunk = {i: asset_names[i] for i in range(asset_start_idx, asset_start_idx + num_assets)}
    return cfg, assets_chunk

def prepare_isaacgym_env_expert(isaacgym_arg_str: str,
                                vulkan_gpu_idx,  # Device to use for graphics (isaac)
                                cuda_gpu_idx,  # Device to use for computation (torch and other environment computation)
                                num_gpu=1,
                                idx=0,  # For multiprocess training. Split assets
                                mode='train',
                                chunk_id=0,  # For evaluation. Split assets
                                num_chunks=1,  # # For evaluation. Split assets
                                expert_run=False,
                                expert_eval_mode=None
                                ):
    """
    Total constraints on assets and environments:
        - The environments must be divisible over the GPUs.
        - The assets must be divisible over the GPUs.
        - The assets per GPU must be divisible over the environments per GPU.
    """
    from MARL_Module.envs.utils.config import get_args, load_cfg, parse_sim_params, set_np_formatting
    from MARL_Module.envs.utils.parse_task import parse_task
    # noinspection PyUnresolvedReferences
    from MARL_Module.envs.utils.process_sarl import process_ppo_pc_pure, process_ppo

    set_np_formatting()

    isaacgym_arg_str = isaacgym_arg_str.replace("cuda:0", f"cuda:{cuda_gpu_idx}")

    args = get_args(isaacgym_arg_str=isaacgym_arg_str)
    args.graphics_device_id = vulkan_gpu_idx
    args.compute_device_id = cuda_gpu_idx
    args.device = 'cuda'
    args.device_id = cuda_gpu_idx
    args.device_ids = [cuda_gpu_idx]

    cfg, cfg_train, logdir = load_cfg(args, expert_run=expert_run)

    cfg["cp"]["device"] = cuda_gpu_idx
    cfg["cp"]["device_ids"] = [cuda_gpu_idx]
    cfg["cp"]["output_device_id"] = f"cuda:{cuda_gpu_idx}"

    if mode == 'train':
        assert chunk_id == 0 and num_chunks == 1
        cfg, assets_chunk = filter_assets(cfg, idx, num_gpu, 'train')
        logdir = logdir + f"_gpu_{idx}"
    else:  # test
        assert idx == 0 and num_gpu == 1
        if expert_eval_mode is None:
            cfg, assets_chunk = filter_assets(cfg, chunk_id, num_chunks, 'val')
        else:
            assert expert_eval_mode in ['train', 'val']
            cfg, assets_chunk = filter_assets(cfg, chunk_id, num_chunks, expert_eval_mode)

    sim_params = parse_sim_params(args, cfg, cfg_train)
    task, env = parse_task(args, cfg, cfg_train, sim_params, None, logdir)
    # expert
    if idx == 0:
        expert = eval('process_{}'.format(args.algo))(args, env, cfg_train, logdir)
        expert.test(args.expert_policy)
    else:
        expert = None

    return env, expert
