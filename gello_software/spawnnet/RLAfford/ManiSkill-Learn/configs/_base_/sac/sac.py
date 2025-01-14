agent = dict(
    type='SAC',
    batch_size=256,
    gamma=0.99,
    update_coeff=0.005,
    alpha=0.2,
    target_update_interval=1,
    automatic_alpha_tuning=True,
    alpha_optim_cfg=dict(type='Adam', lr=0.0003),
)

log_level = 'INFO'

train_mfrl_cfg = dict(
    on_policy=False,
)

rollout_cfg = dict(
    type='Rollout',
    use_cost=False,
    reward_only=False,
    num_procs=1,
)


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