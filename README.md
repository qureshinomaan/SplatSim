## Readme under construction
# SplatSim
SplatSim: Zero-Shot Sim2Real Transfer of RGB Manipulation Policies Using Gaussian Splatting

[Project Page](https://splatsim.github.io) | [Arxiv](https://arxiv.org/abs/2409.10161)



This repository contains the code for the paper "SplatSim". 

## Installation

### Clone this repository
```bash
cd ~/code
git clone --recursive https://github.com/jwang078/SplatSim.git
```

If you accidentally forgot the `--recursive`, run this command to download the files for the `submodules` folder: `git submodule update --init --recursive`

### Create conda env

```bash
conda create -n splatsim python=3.12
conda activate splatsim
```

### Install pytorch


Go to https://pytorch.org/get-started/locally/ to find the right commands for your system. For example, this repo worked with this configuration:
  - torch==2.7.1
  - torchaudio=2.7.1+cu126
  - torchvision=0.22.1+cu126

```bash
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu126
```

### Install submodules

Note: If nothing is inside the `submodules/` folder, run `git submodule update --init --recursive`

```bash
pip install submodules/diff-gaussian-rasterization submodules/pybullet-URDF-models submodules/pybullet-playground-wrapper/ submodules/ghalton

# This needs editable mode for some reason
pip install -e submodules/gaussian-splatting-wrapper

# Seems like -e pip installs must be installed separately
pip install -e submodules/gello_software

# Install dependencies from gello_software
pip install -r submodules/gello_software/requirements.txt

# Allow the install to use your version of pytorch; this takes a few mins for some reason
pip install submodules/simple-knn/ --no-build-isolation
```

QOL to clean up git status
```bash
echo '*.egg-info' >> .git/modules/submodules/ghalton/info/exclude
echo '*.egg-info' >> .git/modules/submodules/pybullet-URDF-models/info/exclude
echo 'build/*' >> .git/modules/submodules/pybullet-URDF-models/info/exclude
```

### Install other dependencies

Install requirements:
```bash
cd ~/code/SplatSim
pip install -r requirements.txt
```

Note: It's important to pip install the `ghalton` submodule in the previous step before installing requirements.txt because for some reason ghalton must be installed from source

### Install this repo as a package

Install in editable mode:

```bash
cd ~/code/SplatSim
pip install -e .
```

## Running the rendering code 
### 1. Download the colmap and gaussian-splatting models from the below links:
- [colmap (test_data)](https://drive.google.com/file/d/14D3fFtaPX4GBe9dSJLKAIvUYlgK7fUxS/view?usp=sharing)
- [gaussian-splats (output)](https://drive.google.com/file/d/1rAUkf7l2ZZqG1Bm3ih6cAO5HCd9dSTO-/view?usp=sharing)
- [trajectories (bc_data/gello)](https://drive.google.com/file/d/1NhSBNYMi51hETAspk6vN7F-Ih1134_lt/view?usp=sharing)

`test_data` is the folder name of the output of colmap. `output` is the folder name of the output of gaussian splat generation. `bc_data/gello` is the folder name of the demo trajectories recorded by one of the scripts in this repo.

Assume below that these files are stored under:

- test_data: /home/yourusername/data/test_data
- output: /home/yourusername/data/output
- bc_data/gello: /home/yourusername/data/bc_data/gello

### 2. Configure the configs to match your folder directory structure

#### Open `configs/object_configs/objects.yaml`. The data you downloaded in step 1 is for the robot `robot_iphone`.

Modify `robot_iphone` as below:
- source_path: /home/yourusername/data/test_data/robot_iphone # Path to a folder you downloaded
- model_path: /home/yourusername/data/output/robot_iphone # Path to a folder you downloaded

#### Open `configs/trajectory_configs.yaml`

Modify as follows:
- trajectory_folder: /home/yourusername/data/bc_data/gello

### 3. Run the rendering script:

Launch the robot server which includes an apple and a plate:
```bash
python scripts/launch_nodes.py --robot sim_ur_pybullet_apple_interactive
```

Wait about 10 seconds for the simulation window to pop up. It will say it is ready to serve.

In another terminal tab, launch a node that will send the recorded trajectories in `/home/yourusername/data/bc_data/gello` to the server so that it will be rendered:
```bash
python scripts/run_env_sim.py --agent replay_trajectory --robot-port 6001
```

A window should pop up that is a rendering of the robot in the pybullet simulation. If you drag the end effector of the robot around in pybullet, it should be reflected in the render.

## Adding a new robot

TODO

## Generating new trajectories

TODO

## A list of TODOs

- [x] ~~Proper instructions for installation. Current instructions might not work.~~
    - [x] Installation instructions should work now.
- [x] ~~Add links to pretrain gaussian-splats and trajectories, so that people can run rendering script.~~
    - [x] Links to pretrain gaussian-splats, colmap and trajectories are added.
- [x] ~~Create a new file for rendering robot and objects, without hardcoding the segmentation and shifting everything to KNN based segmentation.~~
- [x] Clean up the splat folder for only keeping necessary files and easy creation of KNN based segmentation for robots.
- [ ] Documentation for the codebase.
- [ ] Adding new robots (in sim or any other environment).
- [ ] Instructions to generate a trajectory and render it. 
    - [ ] Trajectory format should be specified properly.
- [ ] Clean up the gello folder and only keep files that are necessary. 
 