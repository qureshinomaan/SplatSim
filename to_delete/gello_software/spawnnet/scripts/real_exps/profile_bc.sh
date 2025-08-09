export EXP_PREFIX=place_bag_profile
export EPOCHS=5
export TASK_EE='place_bag'

# # dino
# export CONFIG_NAME=xarm_train_dino
# echo "DINO"
# CUDA_VISIBLE_DEVICES=1 python simple_bc/profile.py --config-name=$CONFIG_NAME \
#   task=$TASK_EE epochs=$EPOCHS num_workers=16 batch_size=8 \
#   experiment=${EXP_PREFIX}_dino \
#   use_proprio=False \
#   dataset.aug_cfg.aug_prob=0.5 \
#   lr=5e-5 action_horizon=4

# # spawn DINO
# export CONFIG_NAME=xarm_train_spawnnet
# echo "Spawn DINO"
# CUDA_VISIBLE_DEVICES=1 python simple_bc/profile.py --config-name=$CONFIG_NAME \
#   task=$TASK_EE epochs=$EPOCHS num_workers=16 batch_size=8 \
#   experiment=${EXP_PREFIX}_spawnnet \
#   use_proprio=False use_depth=False \
#   dataset.aug_cfg.aug_prob=0.5 \
#   lr=3e-4 action_horizon=4

# #r3m
# export CONFIG_NAME=xarm_train_r3m
# echo "R3M"
# CUDA_VISIBLE_DEVICES=1 python simple_bc/profile.py --config-name=$CONFIG_NAME \
#   task=$TASK_EE epochs=$EPOCHS num_workers=16 batch_size=32 \
#   experiment=${EXP_PREFIX}_r3m \
#   use_proprio=False \
#   dataset.aug_cfg.aug_prob=0.5 \
#   lr=5e-5 action_horizon=4

# #spawn r3m
# export CONFIG_NAME=xarm_train_spawnnet
# echo "Spawn R3M"
# CUDA_VISIBLE_DEVICES=1 python simple_bc/profile.py --config-name=$CONFIG_NAME \
#   task=$TASK_EE epochs=$EPOCHS num_workers=4 batch_size=4 \
#   experiment=${EXP_PREFIX}_spawnnet_r3m \
#   use_proprio=False use_depth=False \
#   encoder=spawnnet_r3m \
#   dataset.aug_cfg.aug_prob=0.5 \
#   lr=3e-4 action_horizon=4

# lfs
export CONFIG_NAME=xarm_train_impala
echo "LFS"
CUDA_VISIBLE_DEVICES=1 python simple_bc/profile.py --config-name=$CONFIG_NAME \
  task=$TASK_EE epochs=$EPOCHS num_workers=16 batch_size=128 \
  experiment=${EXP_PREFIX}_impala \
  use_proprio=False \
  dataset.aug_cfg.aug_prob=0.5 \
  lr=3e-4 action_horizon=4

export CONFIG_NAME=xarm_train_impala
echo "LFS large"
CUDA_VISIBLE_DEVICES=1 python simple_bc/profile.py --config-name=$CONFIG_NAME \
  task=$TASK_EE epochs=$EPOCHS num_workers=16 batch_size=128 \
  experiment=${EXP_PREFIX}_impala \
  encoder.large=True \
  use_proprio=False \
  dataset.aug_cfg.aug_prob=0.5 \
  lr=3e-4 action_horizon=4

export CONFIG_NAME=xarm_train_impala
echo "LFS larger"
CUDA_VISIBLE_DEVICES=1 python simple_bc/profile.py --config-name=$CONFIG_NAME \
  task=$TASK_EE epochs=$EPOCHS num_workers=16 batch_size=128 \
  experiment=${EXP_PREFIX}_impala \
  encoder.large=True \
  encoder.larger=True \
  use_proprio=False \
  dataset.aug_cfg.aug_prob=0.5 \
  lr=3e-4 action_horizon=4

