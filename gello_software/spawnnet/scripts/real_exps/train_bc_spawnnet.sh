export EXP_PREFIX=place_bag
export EPOCHS=10
export TASK_EE='place_bag'

export CONFIG_NAME=xarm_train_spawnnet
CUDA_VISIBLE_DEVICES=5 python simple_bc/train.py --multirun --config-name=$CONFIG_NAME \
  task=$TASK_EE epochs=$EPOCHS num_workers=16 batch_size=8 \
  experiment=${EXP_PREFIX}_spawnnet \
  use_proprio=False use_depth=False \
  dataset.aug_cfg.aug_prob=0.5 \
  lr=1e-4,3e-4 action_horizon=4 &