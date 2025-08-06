agent = dict(
    type='BC',
    batch_size=256,
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
