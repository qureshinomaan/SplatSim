log_level = 'INFO'
stack_frame = 1


agent = dict(
    type='TD3_BC',
    batch_size=64,
    gamma=0.95,
    update_coeff=0.005,
    policy_update_interval=2,
    alpha=0.1,
    reward_scale=1,
    policy_cfg=dict(
        type='ContinuousPolicy',
        policy_head_cfg=dict(
            type='DeterministicHead',
            noise_std=0.1,
        ),
        nn_cfg=dict(
            type='LinearMLP',
            norm_cfg=None,
            mlp_spec=['obs_shape', 256, 256, 256, 'action_shape'],
            bias='auto',
            inactivated_output=True,
            linear_init_cfg=dict(
                type='xavier_init',
                gain=1,
                bias=0,
            )
        ),
        optim_cfg=dict(type='Adam', lr=3e-4),
    ),
    value_cfg=dict(
        type='ContinuousValue',
        num_heads=2,
        nn_cfg=dict(
            type='LinearMLP',
            norm_cfg=None,
            bias='auto',
            mlp_spec=['obs_shape + action_shape', 256, 256, 256, 1],
            inactivated_output=True,
            linear_init_cfg=dict(
                type='xavier_init',
                gain=1,
                bias=0,
            )
        ),
        optim_cfg=dict(type='Adam', lr=5e-4),
    ),
    action_noise=0.2,
    noise_clip=0.5,
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

train_mfrl_cfg = dict(
    on_policy=False,
)

env_cfg = dict(
    type='gym',
    unwrapped=False,
    reward_type='dense',
)
