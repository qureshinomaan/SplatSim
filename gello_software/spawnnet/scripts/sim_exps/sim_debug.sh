# For testing other methods: Change to appropriate config name.
export CONFIG_NAME=isaacgym_train_impala

# Leave unchanged.
export ISAACGYM_ARG_STR="--headless --rl_device=cuda:0 --sim_device=cuda:0 --cp_device=cuda:0 --test --use_image_obs=True  --num_envs=1 --num_objs=1"

# For testing other tasks/seeds: change below lines.
TASK=open_door_21
SEED=0

# For testing other methods:
# 1. Copy-paste python command from appropriate method's script.
# 2. Remove --multirun from command.
# 3. Set debug=True and num_gpus=1 within command. 
# 4. Rename experiment=<experiment_name> to desired experiment name.
python RLAfford/dagger/train.py --config-name=$CONFIG_NAME debug=True num_gpus=1 \
    isaacgym_task=$TASK hydra/launcher=$HYDRA_LAUNCHER \
    experiment=sim_lfs_${TASK}_seed$SEED \
    encoder.large=False encoder.larger=False encoder.use_depth=False \
    lr=3e-4 batch_size=24 \
    seed=$SEED