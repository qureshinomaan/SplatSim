## Readme under construction
# SplatSim
SplatSim: Zero-Shot Sim2Real Transfer of RGB Manipulation Policies Using Gaussian Splatting

[Project Page](https://splatsim.github.io) | [Arxiv](https://arxiv.org/abs/2409.10161)



This repository contains the code for the paper "SplatSim". 

## Installation

```bash
conda env create -f environment.yml
conda activate splatsim
pip3 install -r requirements.txt
cd gello
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
- [ ] Clean up the gello folder and only keep files that are necessary. 
- [ ] Instructions to generate a trajectory and render it. 
- [ ] Clean up the splat folder for only keeping necessary files. 
- [ ] Adding new robots and unifying the rendering code for d
 
