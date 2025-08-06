python tools/split_datasets.py ./example_mani_skill_data/OpenCabinetDrawer_1045_link_0-v0_pcd.h5 \
--name OpenCabinetDrawer_1045_link_0 \
--output-folder='./example_mani_skill_data/OpenCabinetDrawer_1045_link_0-v0_pcd_shards/' --num-files=2

python -m tools.run_rl configs/bc/mani_skill_point_cloud_transformer_disk.py --gpu-ids=0 \
--work-dir=./work_dirs/OpenCabinetDrawer_1045_link_0-v0_state_shards --clean-up \
--cfg-options "env_cfg.env_name=OpenCabinetDrawer_1045_link_0-v0"
