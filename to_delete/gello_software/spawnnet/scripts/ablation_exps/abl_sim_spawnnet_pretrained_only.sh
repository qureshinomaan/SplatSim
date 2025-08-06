export CONFIG_NAME=isaacgym_train_spawnnet
export ISAACGYM_ARG_STR="--headless --rl_device=cuda:0 --sim_device=cuda:0 --cp_device=cuda:0 --test --use_image_obs=True"

for TASK in open_door_21
do
    for SEED in 0 1 2
    do
        python RLAfford/dagger/train.py --config-name=$CONFIG_NAME --multirun debug=False num_gpus=2 \
          isaacgym_task=$TASK  hydra/launcher=$HYDRA_LAUNCHER \
          experiment=sim_spawnnet_pretrained_only_${TASK}_seed${SEED} \
          encoder.vit_cfg.freeze_pretrained=True encoder.conv_cfg.version='last_only' encoder.conv_cfg.channel_mask='no_rgbd' \
          lr=3e-4 batch_size=24 \
          seed=$SEED
    done
done