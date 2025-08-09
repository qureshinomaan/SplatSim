_base_ = ['./cql.py']

agent = dict(
    type='CQL',
    automatic_regularization_tuning=False,
    lagrange_thresh=-1,
    alpha_prime=20,
    min_q_weight=1,
    policy_cfg=dict(
        type='ContinuousPolicy',
        policy_head_cfg=dict(
            type='GaussianHead',
            log_sig_min=-20,
            log_sig_max=2,
            epsilon=1e-6
        ),
        nn_cfg=dict(
            type='LinearMLP',
            norm_cfg=None,
            mlp_spec=['obs_shape', 256, 256, 256, 'action_shape * 2'],
            bias='auto',
            inactivated_output=True,
            linear_init_cfg=dict(
                type='xavier_init',
                gain=1,
                bias=0,
            )
        ),
        optim_cfg=dict(type='Adam', lr=3e-5),
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
        optim_cfg=dict(type='Adam', lr=3e-4),
    ),
)

