source ~/.bashrc

. activate spawnnet
export DATASET_DIR=$PWD/dataset
export DATA_DIR=$PWD/data
export PYTHONPATH=$PWD:$PWD/RLAfford:$PWD/RLAfford/MARL_Module/envs/:$PWD/RLAfford/Collision_Predictor_Module/CollisionPredictor/code:$PWD/RLAfford/Collision_Predictor_Module/where2act/code:$PYTHONPATH
export NUMEXPR_MAX_THREADS=16

###### TODO for end user: ######
# 1. export HYDRA_LAUNCHER to be "slurm" (if SLURM cluster) or "basic" (if not a SLURM cluster)
# 2. prepend your /usr/lib/x86_64-linux-gnu/ and <conda envs directory>/spawnnet/lib to LD_LIBRARY_PATH
## Do below according to your cuda-11.7 or greater installation. ##
# 3. export PATH=/usr/local/cuda-11.7/bin:$PATH
# 4. export LD_LIBRARY_PATH=/usr/local/cuda-11.7/lib64:$LD_LIBRARY_PATH
# 5. export CUDA_HOME=/usr/local/cuda-11.7/
################################


