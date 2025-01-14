export EXP_PREFIX=place_bag
export EPOCHS=20
export TASK_EE='place_bag'

export CONFIG_NAME=xarm_train_mvp
CUDA_VISIBLE_DEVICES=1 python simple_bc/train.py --multirun --config-name=$CONFIG_NAME \
  task=$TASK_EE epochs=$EPOCHS num_workers=16 batch_size=8 \
  experiment=${EXP_PREFIX}_mvp \
  use_proprio=False \
  dataset.aug_cfg.aug_prob=0.5 \
  lr=1e-4,5e-4,1e-5,5e-5 action_horizon=4 &