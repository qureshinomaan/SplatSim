# SpawnNet: Learning Generalizable Visuomotor Skills from Pre-trained Networks

[Xingyu Lin](https://xingyu-lin.github.io)\*,
[John So](https://www.johnrso.xyz/)\*,
[Sashwat Mahalingam](https://sashwat-mahalingam.github.io),
[Fangchen Liu](https://fangchenliu.github.io/),
[Pieter Abbeel](https://people.eecs.berkeley.edu/~pabbeel/)

[paper]() | [website](https://xingyu-lin.github.io/spawnnet)

## Setup

For reproducibility, we provide steps and checkpoints to reproduce our simulation DAgger experiments.
For real-world BC experiments, we additionally provide a dataset for
visualization and BC training. For more information regarding the real-world setup,
see [Real World Experiments](#real-world-experiments).

### Create the environment:
```
cd spawnnet
conda create -n spawnnet python==3.8.13 pip
conda activate spawnnet
sudo apt install ffmpeg
pip install -r requirements.txt

# baselines
pip install git+https://github.com/ir413/mvp # MVP
pip install git+https://github.com/facebookresearch/r3m # R3M
```

### Modify `prepare.sh`
`prepare.sh` is a file used to set up the necessary environment variables and library paths. You must modify `prepare.sh` as described in the file's comments.
Make sure to `source prepare.sh` once completed:

```sh
. prepare.sh
```

### Simulation Setup

We run our simulation experiments using [IsaacGym](https://developer.nvidia.com/isaac-gym) with tasks lifted from [RLAfford](https://sites.google.com/view/rlafford/). To set up the IsaacGym environments developed by RLAfford, follow the instructions given at [`RLAfford/README.md`](https://github.com/johnrso/spawnnet/blob/main/RLAfford/README.md).

### PIP Troubleshooting
If when installing any packages, you get a PIP error that `extra_requires` must be a dictionary, consider changing your setuptools version through `pip install setuptools==65.5.0`, then rerunning.

### NVIDIA Troubleshooting: Driver Mismatches/Issues
If you encounter issues with a driver mismatch between your CUDA and NVIDIA Drivers, consider these two steps:

1. Consider adding the `LD_LIBRARY_PATH`, `PATH`, and `CUDA_HOME` changes from `prepare.sh` to your bash profile `~/.bashrc`. This may need a terminal and/or system restart.
    - Typically, legacy CUDA versions may interfere with graphics processes before the variables are updated in `prepare.sh`. This step is meant to resolve that issue.
2. Ensure that your CUDA version >= 11.7 (`nvcc --version`), and that you have a [compatible](https://docs.nvidia.com/deeplearning/cudnn/support-matrix/index.html) NVIDIA-Driver version. Any re-installations may require a system reboot.

## Repository Structure

### `conf`

We use [hydra](https://github.com/facebookresearch/hydra) to manage experiments. Configs correspond exactly to the
module in `simple_bc`.

### `gdict`

This is a library for storing dictionaries of tensors; supports array-like indexing and slicing, and
dictionary-like key indexing. Extracted from [ManiSkill2-Learn](https://github.com/haosulab/ManiSkill2-Learn)

### `simple_bc`

This can be roughly split into 3 modules:
1. **`dataset`**: this loads preprocessed `hdf5` files into `GDict` structs.
2. **`encoder`**: this processes inputs into latent vectors.
3. **`policy`**: these are learning algorithms to output actions.

The network modules all follow interfaces defined in `_interfaces`. To add a new network, implement the abstract methods
in each interface (see
[`encoder/impala.py`](https://github.com/johnrso/spawnnet/blob/main/simple_bc/encoder/impala.py)
for an example), add the network to the module `__init__.py` file (see
[`encoder/__init__.py`](https://github.com/johnrso/spawnnet/blob/main/simple_bc/encoder/__init__.py))
and define a hydra configuration in root's `conf` (see
[`conf/encoder/impala.yaml`](https://github.com/johnrso/spawnnet/blob/main/conf/encoder/impala.yaml)).

Additionally, we provide scripts for training and evaluating policies under `train.py` and `eval.py`.

## Simulation Experiments

__Note__: Before running experiments in a terminal, be sure to `source prepare.sh` first.

### Training
There are two tasks in simulation, Open Drawer and Open Door. The IsaacGym configurations for both tasks can be found under `RLAfford/cfg/open_door_expert.yaml` and `RLAfford/cfg/open_drawer_expert.yaml`, respectively.

After setting up everything, set **only** `WHICH_GPUS` if in non-SLURM, i.e. basic, mode. Do **not** set anything for SLURM mode, the launcher will handle it. This is due to Vulkan/PyTorch differences in GPU indexing.

An example of training `SpawnNet` DAgger on the `Open Drawer` task is found in `scripts/sim_exps/spawnnet_exp.sh`.

1. Make sure to specify the `ISAACGYM_ARG_STR` as an environment variable (it should be the exact same value as the example).
2. For the drawer task, use `isaacgym_task=open_drawer`, and for the door task, use `isaacgym_task=open_door_21`.
3. **Optional**: Our framework splits 21 training assets among the allocated GPUs. Each asset has a corresponding simulation environment that's assigned to the same GPU as the asset. By default, each GPU gets `floor(21 / num_gpus)` assets (with the remainder assets going to the last GPU). If you wish to split the assets differently, set the variable `TRAIN_ASSET_SPLIT` as follows when kicking off the `train.py` script:
    ```sh
    TRAIN_ASSET_SPLIT=<# assets on 0th GPU>,<# assets on 1st GPU>,<# assets on 2nd GPU>,...
    ```
    When a larger model is being trained, the primary GPU (where the model resides) may run into CUDA memory issues from sharing space with too many simulation environments. The other GPUs may have space to load more environments. This fix is helpful for that case.
    *Note that this custom asset splitting only applies for training.*

We provide entrypoints for each experiment in `scripts/sim_exps`.

#### Debugging Training
We provide a script, `scripts/sim_exps/sim_debug.sh`, to assist with debugging training in simulation. This script enforces only one environment, one GPU, to be used.

You can run the script as is to test that the `spawnnet` simulation framework is functioning correctly. You can also test different methods, tasks, and seeds by following the comments in the script. Leave the `ISAACGYM_ARG_STR` as is, to ensure only one environment is loaded (for faster testing).

**Note: This script always runs with only one GPU.**


### Evaluation

If you're in SLURM mode, evaluations are handled automatically by our training script and can be found under the `eval` folder of your run. Statistics will be listed under `summary.csv`.

Otherwise (basic mode), due to memory issues with IsaacGym, evaluations must be handled manually, on any **single GPU**. The syntax is the same regardless of the experiment done:

```sh
export ISAACGYM_ARG_STR="--headless --rl_device=cuda:0 --sim_device=cuda:0 --cp_device=cuda:0 --test --use_image_obs=True"
WHICH_GPUS=0 python RLAfford/dagger/eval.py <your exp dir> --chunk_id 0 --num_chunks 1 --mode basic
```

and results get saved the same way as SLURM.

## Real World Experiments
For Real World BC Experiments, the demonstration set for the Place Bag task can be found at [this Google Drive link](https://drive.google.com/uc?id=1A4RGlKM7GDalBAA4jKTmjcMyUzkwFBJW). You can download this with [gdown](https://github.com/wkentaro/gdown). After downloading, place the unzipped directory into `/dataset`.

Similarly to simulation, we provide entry points under `scripts/real_exps`.

### Visualizing pre-trained feature attention
After running a SpawnNet experiment, visualizations of the adapter features can be found under the run's directory, which looks like:
```sh
/data/local/0627_place_bag_spawnnet_2050/0/visualization_best
```


### Adding Tasks

To add tasks, please refer to `simple_bc/constants.py`, and follow the format for either `BC_DATASET` or `ISAACGYM_TASK`.

## Acknowledgements

The `gdict` library is adopted from [`ManiSkill2-Learn`](https://github.com/haosulab/ManiSkill2-Learn). Additionally, we use tasks and assets from [`RLAfford`](https://github.com/hyperplane-lab/RLAfford).

THIS SOFTWARE AND/OR DATA WAS DEPOSITED IN THE BAIR OPEN RESEARCH COMMONS REPOSITORY ON Jul. 7., 2022.
