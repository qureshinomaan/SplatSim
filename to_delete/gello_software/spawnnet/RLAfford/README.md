# RLAfford

Documentation pulled from original [RLAfford](https://github.com/hyperplane-lab/RLAfford/) repository, modified for `spawnnet` use case.

## Requirements
RLAfford is tested in NVIDIA-driver version $\geq$ 515, cuda Version $\geq$ 11.7 and  python $\geq$ 3.8 environment can run successfully, if the version is not correct may lead to errors, such as `segmentation fault`.

Some dependencies can be installed by running, within the `RLAfford` directory,

```sh
pip install -r ./requirements.txt
pip install torch==2.0.0 torchvision==0.15.1 torchaudio==2.0.1 sapien==2.2.1 hydra-core==0.11.3
```

### [Isaac Gym](https://developer.nvidia.com/isaac-gym)

RLAfford framework is implemented on Isaac Gym simulator, the version used is Preview Release 4. Download IsaacGym using [this link](https://developer.nvidia.com/isaac-gym). Then, after going to `isaacgym/docs/index.html` -> "Installation", follow "Install in an existing Python environment" and "Simple example."

### [Pointnet2](https://github.com/daerduoCarey/where2act/tree/main/code)

Install pointnet++ manually.

```sh
cd spawnnet/RLAfford/Pointnet2_PyTorch
pip install -r requirements.txt
pip install -e .
```

Finally, run the following to install other packages.

```sh
# make sure you are at the RLAfford directory.
pip install -r requirements.txt
pip install torch==2.0.0 torchvision==0.15.1 torchaudio==2.0.1 sapien==2.2.1 hydra-core==0.11.3
```

### [Maniskill-Learn](https://github.com/haosulab/ManiSkill-Learn)

```sh
cd spawnnet/RLAfford/ManiSkill-Learn/
pip install -e .
pip install hydra-core --upgrade
```

## SLURM GPU Mapping
If using a SLURM cluster, modify `gpu_info.py`. Modify `UUID_DICT` to map the hostname of the cluster to a dictionary of UUID to GPU-index mappings. For example:

```python
UUID_DICT={'hostname1': {'0000-0000...': 0, '1111-1111...': 1}, 'hostname2': {...}}
```

## Dataset Preparation

Download the dataset from [google drive](https://drive.google.com/drive/folders/1FyTuz17uSmAbVSmJUbgb-7OgRM5TalCK?usp=sharing) and extract it. Move the `assets` folder under `spawnnet`. The dataset includes objects from SAPIEN dataset along with additional information processed by RLAfford.

## Expert Policies

We use the DAgger method to train our BC policies in simulation environments. For this, you require access to expert policies trained using PPO. The expert policies can be found [here](https://drive.google.com/uc?id=1MgVZ_0-ExGDHkVJ_aMyiYJFBIwRxRjlx). Download and extract the ZIP and move the `open_door` and `open_drawer` folders to `./data/yiran_pretrained` under `spawnnet`. You can download the ZIP using `gdown`.

If you wish to train expert policies from scratch, please refer to the original RLAfford repository, specifically [Experiments.md](https://github.com/hyperplane-lab/RLAfford/blob/main/Experiments.md). The tasks were `Open Drawer` and `Open Door`, the observation mode used was `CP Map`, and the appropriate configuration files needed will be found on the original repository.

## Draw the Pointcloud

RLAfford's authors use Mitsuba3 to draw pointcloud and affordance map. Scripts can be accessed in the repo [Visualization](https://github.com/GengYiran/Draw_PointCloud).

## Cite

```latex
@article{geng2022end,
  title={End-to-End Affordance Learning for Robotic Manipulation},
  author={Geng, Yiran and An, Boshi and Geng, Haoran and Chen, Yuanpei and Yang, Yaodong and Dong, Hao},
  journal={International Conference on Robotics and Automation (ICRA)},
  year={2023}
}
```

Email of author of original RLAfford repository: hao.dong@pku.edu.cn
