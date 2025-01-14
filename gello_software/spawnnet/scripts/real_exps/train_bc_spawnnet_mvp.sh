export EXP_PREFIX=place_bag
export EPOCHS=10
export TASK_EE='place_bag'

export CONFIG_NAME=xarm_train_spawnnet
CUDA_VISIBLE_DEVICES=5 python simple_bc/train.py --multirun --config-name=$CONFIG_NAME \
  task=$TASK_EE epochs=$EPOCHS num_workers=4 batch_size=4 \
  experiment=${EXP_PREFIX}_spawnnet_mvp \
  use_proprio=False use_depth=True,False \
  encoder=spawnnet_mvp \
  dataset.aug_cfg.aug_prob=0.5 \
  lr=1e-4,3e-4 action_horizon=4 &