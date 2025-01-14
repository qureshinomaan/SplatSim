agent = dict(
    type='TD3_BC',
    batch_size=256,
    gamma=0.95,
    update_coeff=0.005,
    policy_update_interval=2,
    alpha=2.5,
)

log_level = 'INFO'

eval_cfg = dict(
    type='Evaluation',
    num=10,
    num_procs=1,
    use_hidden_state=False,
    start_state=None,
    save_traj=True,
    save_video=True,
    use_log=False,
)

train_mfrl_cfg = dict(
    on_policy=False,
)
