export CONFIG_NAME=isaacgym_train_spawnnet
export ISAACGYM_ARG_STR="--headless --rl_device=cuda:0 --sim_device=cuda:0 --cp_device=cuda:0 --test --use_image_obs=True"

for TASK in open_door_21
do
    for SEED in 1 2 0
    do
        python RLAfford/dagger/train.py --config-name=$CONFIG_NAME --multirun debug=False num_gpus=2 \
          isaacgym_task=$TASK hydra/launcher=$HYDRA_LAUNCHER \
          experiment=sim_isaacgym_spawnnet_no_dense_${TASK}_seed${SEED} \
          encoder.vit_cfg.freeze_pretrained=True encoder.conv_cfg.use_dense=False \
          lr=3e-4 batch_size=24 \
          seed=$SEED
    done
done