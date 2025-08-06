log_level = 'INFO'
stack_frame = 1


agent = dict(
    type='CQL',
    batch_size=32,
    gamma=0.95,

    update_coeff=0.005,
    target_update_interval=1,
    num_action_sample=8,
    lagrange_thresh=-1,
    alpha=0.2,
    alpha_prime=20,
    automatic_alpha_tuning=True,
    automatic_regularization_tuning=False,
    alpha_optim_cfg=dict(type='Adam', lr=3e-5),
    alpha_prime_optim_cfg=dict(type='Adam', lr=3e-4),
    temperature=1,
    min_q_weight=1,
    min_q_with_entropy=False,
    target_q_with_entropy=True,
    forward_block=4,

    policy_cfg=dict(
        type='ContinuousPolicy',
        policy_head_cfg=dict(
            type='GaussianHead',
            log_sig_min=-20,
            log_sig_max=2,
            epsilon=1e-6
        ),
        nn_cfg=dict(
            type='PointNetV0',
            conv_cfg=dict(
                type='ConvMLP',
                norm_cfg=None,
                mlp_spec=['agent_shape + pcd_all_channel', 256, 512],
                bias='auto',
                inactivated_output=False,
                conv_init_cfg=dict(
                    type='xavier_init',
                    gain=1,
                    bias=0,
                )
            ),
            mlp_cfg=dict(
                type='LinearMLP',
                norm_cfg=None,
                mlp_spec=[512 * stack_frame, 256, 'action_shape * 2'],
                bias='auto',
                inactivated_output=True,
                linear_init_cfg=dict(
                    type='xavier_init',
                    gain=1,
                    bias=0,
                )
            ),
            subtract_mean_coords=True,
            max_mean_mix_aggregation=True,
            stack_frame=stack_frame,
        ),
        optim_cfg=dict(type='Adam', lr=3e-4),
    ),
    value_cfg=dict(
        type='ContinuousValue',
        num_heads=2,
        nn_cfg=dict(
            type='PointNetV0',
            conv_cfg=dict(
                type='ConvMLP',
                norm_cfg=None,
                mlp_spec=['agent_shape + action_shape + pcd_all_channel', 256, 512],
                bias='auto',
                inactivated_output=False,
                conv_init_cfg=dict(
                    type='xavier_init',
                    gain=1,
                    bias=0,
                )
            ),
            mlp_cfg=dict(
                type='LinearMLP',
                norm_cfg=None,
                mlp_spec=[512 * stack_frame, 256, 1],
                bias='auto',
                inactivated_output=True,
                linear_init_cfg=dict(
                    type='xavier_init',
                    gain=1,
                    bias=0,
                )
            ),
            subtract_mean_coords=True,
            max_mean_mix_aggregation=True,
            stack_frame=stack_frame,
        ),
        optim_cfg=dict(type='Adam', lr=3e-4),
    ),
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
    stack_frame=stack_frame,
    obs_mode='pointcloud',
    reward_type='dense',
)
