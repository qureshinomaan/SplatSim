# split_datasets.py is necessary to decorrelate and randomly shuffle the training data, so that during training, a batch of training data does not belong to a single h5 file
# change num_files based on the memory available on your device
python tools/split_datasets.py ./full_mani_skill_data/OpenCabinetDrawer/ --name OpenCabinetDrawer --output-folder='./full_mani_skill_data/OpenCabinetDrawer_shards/' --num-files=0

# change the environment name to OpenCabinetDoor, OpenCabinetDrawer, PushChair, or MoveBucket
# change the network config
# increase eval_cfg.num_procs for parallel evaluation
python -m tools.run_rl configs/bc/mani_skill_point_cloud_transformer_disk.py --gpu-ids=0 \
        --work-dir=./work_dirs/bc_transformer_drawer/ \
        --cfg-options "train_mfrl_cfg.total_steps=150000" "train_mfrl_cfg.init_replay_buffers=./full_mani_skill_data/OpenCabinetDrawer_shards/" \
        "env_cfg.env_name=OpenCabinetDrawer-v0" "eval_cfg.num=300" "eval_cfg.num_procs=5" "train_mfrl_cfg.n_eval=150000"
