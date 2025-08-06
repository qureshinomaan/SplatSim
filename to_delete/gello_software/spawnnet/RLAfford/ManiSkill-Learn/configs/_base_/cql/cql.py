agent = dict(
    type='CQL',
    batch_size=256,
    gamma=0.95,
    update_coeff=0.005,
    target_update_interval=1,
    num_action_sample=10,
    lagrange_thresh=10,
    alpha=0.2,
    alpha_prime=5,
    automatic_alpha_tuning=True,
    automatic_regularization_tuning=True,
    alpha_optim_cfg=dict(type='Adam', lr=3e-5),
    alpha_prime_optim_cfg=dict(type='Adam', lr=3e-4),
    temperature=1,
    min_q_weight=1,
    min_q_with_entropy=True,
    target_q_with_entropy=True,
    forward_block=1,
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
