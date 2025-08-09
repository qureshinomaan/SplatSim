_base_ = ['./bcq.py']

agent = dict(
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
    policy_vae_cfg=dict(
        type='VAEPolicy',
        policy_head_cfg=dict(
            type='DeterministicHead',
            noise_std=1e-5),
        nn_cfg=dict(
            type='CVAE',
            latent_dim='action_shape * 2',
            encoder_cfg=dict(
                type='LinearMLP',
                norm_cfg=None,
                bias='auto',
                mlp_spec=['obs_shape + action_shape', 256, 256, 256, 'action_shape * 4'],
                inactivated_output=True,
            ),
            decoder_cfg=dict(
                type='LinearMLP',
                norm_cfg=None,
                bias='auto',
                mlp_spec=['obs_shape + action_shape * 2', 256, 256, 256, 'action_shape'],
                inactivated_output=True,
            ),
            log_sig_min=-4,
            log_sig_max=15,
        ),
        optim_cfg=dict(type='Adam', lr=3e-4, weight_decay=5e-6),
    ),
)
