## Readme under construction
# SplatSim
SplatSim: Zero-Shot Sim2Real Transfer of RGB Manipulation Policies Using Gaussian Splatting

[Project Page](https://splatsim.github.io) | [Arxiv](https://arxiv.org/abs/2409.10161)



This repository contains the code for the paper "SplatSim". 

## Installation

```bash
conda env create -f environment.yml
cd gello
pip install -e .
```

## A general description of the codebase

### 1. The most important file is render_fk_all_highres.py

This file contains the code for rendering forward kinematics of the robot as well as the objects in the scene. 

### 2. The folder "gello" contains the code for trajectory generation, which is then rendered using the code in the first file. 

Currently, there is a lot of code here, need to clean it up and only keep files that are necessary. 

### 3. The folder "splat" contains the code for KNN logic and some other utilities.

Very unorganized, will need to clean it up. 



## A list of TODOs

- [ ] Proper instructions for installation. Current instructions might not work. 
- [ ] Clean up the gello folder and only keep files that are necessary. 
- [ ] Instructions to generate a trajectory and render it. 
- [ ] Clean up the splat folder for only keeping necessary files. 
- [ ] Adding new robots and unifying the rendering code for d
 
