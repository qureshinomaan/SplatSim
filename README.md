## Readme under construction
# SplatSim
SplatSim: Zero-Shot Sim2Real Transfer of RGB Manipulation Policies Using Gaussian Splatting

[Project Page](https://splatsim.github.io) | [Arxiv](https://arxiv.org/abs/2409.10161)



This repository contains the code for the paper "SplatSim". 

## Installation

### Clone this repository
```bash
cd ~/code
git clone TODO --recursive
```

### Create conda env

```bash
conda create -n splatsim python=3.12
```

### Install pytorch


Go to https://pytorch.org/get-started/locally/ to find the right commands for your system. For example, this repo worked with this configuration:
  - torch==2.7.1
  - torchaudio=2.7.1+cu128
  - torchvision=0.22.1+cu128

```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
```

### Install ghalton

This is a dependency of pybullet-planning which has a hard time installing on its own. Install from source:

```bash
cd ~/code
git clone https://github.com/fmder/ghalton.git
cd ghalton
pip install .
cd ~/code/SplatSim
```

### Install other dependencies

Install requirements:
```bash
cd ~/code/SplatSim
pip install -r requirements.txt
```

### Install this repo as a package

```bash
cd ~/code/SplatSim
pip install .
```




```bash
conda env create -f environment.yml
conda activate splatsim
pip3 install -r requirements.txt
cd gello
pip install -e .
cd pybullet-URDF-models
pip install -e .
```

## Running the rendering code 
### 1. Download the colmap and gaussian-splatting models from the below links:
- [colmap (test_data)](https://drive.google.com/file/d/14D3fFtaPX4GBe9dSJLKAIvUYlgK7fUxS/view?usp=sharing)
- [gaussian-splats (output)](https://drive.google.com/file/d/1rAUkf7l2ZZqG1Bm3ih6cAO5HCd9dSTO-/view?usp=sharing)
- [trajectories (bc_data/gello)](https://drive.google.com/file/d/1NhSBNYMi51hETAspk6vN7F-Ih1134_lt/view?usp=sharing)

### 2. Run the rendering script:

```bash
render_fk_all_highres.py -s /path/to/test_data/robot_iphone -m /path/to/output/robot_iphone --objects plastic_apple --traj_folder /path/to/bc_data/gello
```

## A general description of the codebase

### 1. The most important file is render_fk_all_highres.py

This file contains the code for rendering forward kinematics of the robot as well as the objects in the scene. 

### 2. The folder "gello" contains the code for trajectory generation, which is then rendered using the code in the first file. 

Currently, there is a lot of code here, need to clean it up and only keep files that are necessary. 

### 3. The folder "splat" contains the code for KNN logic and some other utilities.

Very unorganized, will need to clean it up. 



## A list of TODOs

- [x] ~~Proper instructions for installation. Current instructions might not work.~~
    - [x] Installation instructions should work now.
- [x] ~~Add links to pretrain gaussian-splats and trajectories, so that people can run rendering script.~~
    - [x] Links to pretrain gaussian-splats, colmap and trajectories are added.
- [x] Create a new file for rendering robot and objects, without hardcoding the segmentation and shifting everything to KNN based segmentation.
- [x] Clean up the splat folder for only keeping necessary files and easy creation of KNN based segmentation for robots.
- [ ] Documentation for the codebase.
- [ ] Adding new robots (in sim or any other environment).
- [ ] Instructions to generate a trajectory and render it. 
    - [ ] Trajectory format should be specified properly.
- [ ] Clean up the gello folder and only keep files that are necessary. 
 