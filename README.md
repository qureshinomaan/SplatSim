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
git submodule init
git submodule update
```

### Create conda env

```bash
conda create -n splatsim python=3.12
```

### Install pytorch


Go to https://pytorch.org/get-started/locally/ to find the right commands for your system. For example, this repo worked with this configuration:
  - torch==2.7.1
  - torchaudio=2.7.1+cu126
  - torchvision=0.22.1+cu126

```bash
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu126
```

### Install other dependencies

Install requirements:
```bash
cd ~/code/SplatSim
pip install -r requirements.txt
```

### Install submodules

TODO download submodules, maybe with --recursive

Change `submodules/gello_software` to install as a package by changing this in `submodules/gello_software/setup.py`:
- From `packages=setuptools.find_packages(),`
- To: `packages=setuptools.find_packages(include=["gello", "gello.*"]),`

Add `submodules/gaussian-splatting-wrapper/setup.py` and `submodules/gaussian-splatting-wrapper/gaussian_splatting/__init__.py` to gaussian-splatting repo
```
from setuptools import setup, find_packages

setup(
    name="gaussian_splatting",
    version="0.1",
    packages=find_packages(include=["scene", "scene.*", "utils", "utils.*"]),
)
```

```
import os
import sys

# Allow relative imports like `from utils...` to work by appending parent directory
_module_path = os.path.dirname(__file__)
sys.path.insert(0, _module_path)
```

Note that this has to be installed in editable mode `-e`
```bash
pip install -e submodules/gaussian-splatting-wrapper
```

And 

Install submodules and their dependencies

pip install submodules/gello_software
pip install -r submodules/gello_software/requirements.txt

```
# Allow the install to use your version of pytorch; this takes a few mins for some reason
pip install submodules/simple-knn/ --no-build-isolation
```

```
pip install submodules/diff-gaussian-rasterization
```

```
pip install submodules/pybullet-URDF-models
```

Pybullet playground
Add `submodules/pybullet-playground-wrapper/setup.py`
```
from setuptools import setup, find_packages

setup(
    name="pybullet_playground",
    version="0.1.0",
    packages=find_packages(include=["pybullet_playground", "pybullet_playground/*"]),
)
```

```
pip install submodules/pybullet-playground-wrapper/
```

QOL for git status
```bash
echo '*.egg-info' >> .git/modules/submodules/ghalton/info/exclude
echo 'build/*' >> .git/modules/submodules/ghalton/info/exclude
echo '*.egg-info' >> .git/modules/submodules/pybullet-URDF-models/info/exclude
echo 'build/*' >> .git/modules/submodules/pybullet-URDF-models/info/exclude
```

### Install this repo as a package

Install in editable mode:

```bash
cd ~/code/SplatSim
pip install -e .
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
 