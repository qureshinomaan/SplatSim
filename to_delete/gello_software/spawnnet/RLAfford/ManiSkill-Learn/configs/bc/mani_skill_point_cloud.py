_base_ = ['../_base_/bc/bc_mani_skill_pointnet.py']


env_cfg = dict(
    type='gym',
    env_name='OpenCabinetDrawer_1045_link_0-v0',
)


replay_cfg = dict(
    type='ReplayMemory',
    capacity=1000000,
)

train_mfrl_cfg = dict(
    total_steps=50000,
    warm_steps=0,
    n_steps=0,
    n_updates=500,
    n_eval=50000,
    n_checkpoint=50000,
    init_replay_buffers='./example_mani_skill_data/OpenCabinetDrawer_1045_link_0-v0_pcd.h5',
)

eval_cfg = dict(
    num=10,
    num_procs=1,
    use_hidden_state=False,
    start_state=None,
    save_traj=True,
    save_video=False,
    use_log=False,
)
