export CONFIG_NAME=isaacgym_train_impala
export ISAACGYM_ARG_STR="--headless --rl_device=cuda:0 --sim_device=cuda:0 --cp_device=cuda:0 --test --use_image_obs=True"

for TASK in open_door_21 open_drawer
do
    for SEED in 0 1 2
    do
        python RLAfford/dagger/train.py --config-name=$CONFIG_NAME --multirun debug=False num_gpus=2 \
          isaacgym_task=$TASK hydra/launcher=$HYDRA_LAUNCHER \
          experiment=sim_lfs_aug_${TASK}_seed$SEED \
          encoder.large=True encoder.larger=True encoder.use_depth=False \
          dataset.aug_cfg.aug_prob=0.5 \
          lr=1e-4 \
          seed=$SEED
    done
done