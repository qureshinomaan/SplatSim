python -m tools.run_rl configs/bc/mani_skill_point_cloud_transformer.py --gpu-ids=0 --evaluation \
--work-dir=./test/OpenCabinetDrawer_1045_link_0-v0_pcd \
--resume-from=./example_mani_skill_data/OpenCabinetDrawer_1045_link_0-v0_PN_Transformer.ckpt \
--cfg-options "env_cfg.env_name=OpenCabinetDrawer_1045_link_0-v0" "eval_cfg.save_video=True" "eval_cfg.num=100" "eval_cfg.num_procs=10" "eval_cfg.use_log=True"

python -m tools.run_rl configs/bc/mani_skill_state.py --gpu-ids=0 --evaluation \
--work-dir=./test/OpenCabinetDrawer_1045_link_0-v0_state \
--resume-from=./example_mani_skill_data/OpenCabinetDrawer_1045_link_0-v0_mlp.ckpt \
--cfg-options "env_cfg.env_name=OpenCabinetDrawer_1045_link_0-v0" "eval_cfg.save_video=True" "eval_cfg.num=100" "eval_cfg.num_procs=10" "eval_cfg.use_log=True"