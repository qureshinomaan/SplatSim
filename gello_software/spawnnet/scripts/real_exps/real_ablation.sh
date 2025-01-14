export EXP_PREFIX=0630_place_bag_abl
export EPOCHS=10
export TASK_EE='place_bag'

export CONFIG_NAME=xarm_train_spawnnet
CUDA_VISIBLE_DEVICES=1 python simple_bc/train.py --multirun --config-name=$CONFIG_NAME \
  task=$TASK_EE epochs=$EPOCHS num_workers=16 batch_size=8 \
  experiment=${EXP_PREFIX}_spawnnet_ablations \
  use_proprio=False \
  dataset.aug_cfg.aug_prob=0.5 \
  lr=1e-4,3e-4,5e-4 action_horizon=4 \
  encoder.conv_cfg.use_dense=True,False \
  encoder.conv_cfg.channel_mask=default,no_rgbd \
  encoder.conv_cfg.version=default,last_only &