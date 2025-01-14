log_level = 'INFO'
stack_frame = 1
num_heads = 4

agent = dict(
    type='BCQ',
    gamma=0.95,
    batch_size=64,
    update_coeff=0.005,
    lmbda=0.75,
    target_update_interval=1,

    value_cfg=dict(
        type='ContinuousValue',
        num_heads=2,
        nn_cfg=dict(
            type='PointNetWithInstanceInfoV0',
            stack_frame=stack_frame,
            num_objs='num_objs',
            pcd_pn_cfg=dict(
                type='PointNetV0',
                conv_cfg=dict(
                    type='ConvMLP',
                    norm_cfg=None,
                    mlp_spec=['agent_shape + pcd_xyz_rgb_channel + action_shape', 192, 192],
                    bias='auto',
                    inactivated_output=True,
                    conv_init_cfg=dict(type='xavier_init', gain=1, bias=0),
                ),
                mlp_cfg=dict(
                    type='LinearMLP',
                    norm_cfg=None,
                    mlp_spec=[192 * stack_frame, 192, 192],
                    bias='auto',
                    inactivated_output=True,
                    linear_init_cfg=dict(type='xavier_init', gain=1, bias=0),
                ),
                subtract_mean_coords=True,
                max_mean_mix_aggregation=True
            ),
            state_mlp_cfg=dict(
                type='LinearMLP',
                norm_cfg=None,
                mlp_spec=['agent_shape + action_shape', 192, 192],
                bias='auto',
                inactivated_output=True,
                linear_init_cfg=dict(type='xavier_init', gain=1, bias=0),
            ),                    
            transformer_cfg=dict(
                type='TransformerEncoder',
                block_cfg=dict(
                    attention_cfg=dict(
                        type='MultiHeadSelfAttention',
                        embed_dim=192,
                        num_heads=num_heads,
                        latent_dim=32,
                        dropout=0.1,
                    ),
                    mlp_cfg=dict(
                        type='LinearMLP',
                        norm_cfg=None,
                        mlp_spec=[192, 768, 192],
                        bias='auto',
                        inactivated_output=True,
                        linear_init_cfg=dict(type='xavier_init', gain=1, bias=0),
                    ),
                    dropout=0.1,
                ),
                pooling_cfg=dict(
                    embed_dim=192,
                    num_heads=num_heads,
                    latent_dim=32,
                ),
                mlp_cfg=None,
            ),
            final_mlp_cfg=dict(
                type='LinearMLP',
                norm_cfg=None,
                mlp_spec=[192, 128, 1],
                bias='auto',
                inactivated_output=True,
                linear_init_cfg=dict(type='xavier_init', gain=1, bias=0),
            ),
        ),
        optim_cfg=dict(type='Adam', lr=5e-4, weight_decay=5e-6),
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
                type='PointNetWithInstanceInfoV0',
                stack_frame=stack_frame,
                num_objs='num_objs',
                pcd_pn_cfg=dict(
                    type='PointNetV0',
                    conv_cfg=dict(
                        type='ConvMLP',
                        norm_cfg=None,
                        mlp_spec=['agent_shape + pcd_xyz_rgb_channel + action_shape', 192, 192],
                        bias='auto',
                        inactivated_output=True,
                        conv_init_cfg=dict(type='xavier_init', gain=1, bias=0),
                    ),
                    mlp_cfg=dict(
                        type='LinearMLP',
                        norm_cfg=None,
                        mlp_spec=[192 * stack_frame, 192, 192],
                        bias='auto',
                        inactivated_output=True,
                        linear_init_cfg=dict(type='xavier_init', gain=1, bias=0),
                    ),
                    subtract_mean_coords=True,
                    max_mean_mix_aggregation=True
                ),
                state_mlp_cfg=dict(
                    type='LinearMLP',
                    norm_cfg=None,
                    mlp_spec=['agent_shape + action_shape', 192, 192],
                    bias='auto',
                    inactivated_output=True,
                    linear_init_cfg=dict(type='xavier_init', gain=1, bias=0),
                ),                               
                transformer_cfg=dict(
                    type='TransformerEncoder',
                    block_cfg=dict(
                        attention_cfg=dict(
                            type='MultiHeadSelfAttention',
                            embed_dim=192,
                            num_heads=num_heads,
                            latent_dim=32,
                            dropout=0.1,
                        ),
                        mlp_cfg=dict(
                            type='LinearMLP',
                            norm_cfg=None,
                            mlp_spec=[192, 768, 192],
                            bias='auto',
                            inactivated_output=True,
                            linear_init_cfg=dict(type='xavier_init', gain=1, bias=0),
                        ),
                        dropout=0.1,
                    ),
                    pooling_cfg=dict(
                        embed_dim=192,
                        num_heads=num_heads,
                        latent_dim=32,
                    ),
                    mlp_cfg=None,
                ),
                final_mlp_cfg=dict(
                    type='LinearMLP',
                    norm_cfg=None,
                    mlp_spec=[192, 192, 'action_shape * 4'],
                    bias='auto',
                    inactivated_output=True,
                    linear_init_cfg=dict(type='xavier_init', gain=1, bias=0),
                ),
            ),
            decoder_cfg=dict(
                type='PointNetWithInstanceInfoV0',
                stack_frame=stack_frame,
                num_objs='num_objs',
                pcd_pn_cfg=dict(
                    type='PointNetV0',
                    conv_cfg=dict(
                        type='ConvMLP',
                        norm_cfg=None,
                        mlp_spec=['agent_shape + pcd_xyz_rgb_channel + action_shape * 2', 192, 192],
                        bias='auto',
                        inactivated_output=True,
                        conv_init_cfg=dict(type='xavier_init', gain=1, bias=0),
                    ),
                    mlp_cfg=dict(
                        type='LinearMLP',
                        norm_cfg=None,
                        mlp_spec=[192 * stack_frame, 192, 192],
                        bias='auto',
                        inactivated_output=True,
                        linear_init_cfg=dict(type='xavier_init', gain=1, bias=0),
                    ),
                    subtract_mean_coords=True,
                    max_mean_mix_aggregation=True
                ),
                state_mlp_cfg=dict(
                    type='LinearMLP',
                    norm_cfg=None,
                    mlp_spec=['agent_shape + action_shape * 2', 192, 192],
                    bias='auto',
                    inactivated_output=True,
                    linear_init_cfg=dict(type='xavier_init', gain=1, bias=0),
                ),                                
                transformer_cfg=dict(
                    type='TransformerEncoder',
                    block_cfg=dict(
                        attention_cfg=dict(
                            type='MultiHeadSelfAttention',
                            embed_dim=192,
                            num_heads=num_heads,
                            latent_dim=32,
                            dropout=0.1,
                        ),
                        mlp_cfg=dict(
                            type='LinearMLP',
                            norm_cfg=None,
                            mlp_spec=[192, 768, 192],
                            bias='auto',
                            inactivated_output=True,
                            linear_init_cfg=dict(type='xavier_init', gain=1, bias=0),
                        ),
                        dropout=0.1,
                    ),
                    pooling_cfg=dict(
                        embed_dim=192,
                        num_heads=num_heads,
                        latent_dim=32,
                    ),
                    mlp_cfg=None,
                ),
                final_mlp_cfg=dict(
                    type='LinearMLP',
                    norm_cfg=None,
                    mlp_spec=[192, 192, 'action_shape'],
                    bias='auto',
                    inactivated_output=True,
                    linear_init_cfg=dict(type='xavier_init', gain=1, bias=0),
                ),
            ),
            log_sig_min=-4,
            log_sig_max=15,
        ),
        optim_cfg=dict(type='Adam', lr=3e-4, weight_decay=5e-6),
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
