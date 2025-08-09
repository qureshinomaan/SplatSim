_base_ = ['./bc.py']

agent = dict(
    policy_cfg=dict(
        type='ContinuousPolicy',
        policy_head_cfg=dict(
            type='DeterministicHead',
            noise_std=1e-5,
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
        optim_cfg=dict(type='Adam', lr=1e-3),
    ),
)

