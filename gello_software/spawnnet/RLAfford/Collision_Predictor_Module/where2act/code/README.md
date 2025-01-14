# Experiments
This folder includes the codebase for Where2Act simulator and experiments.

## Before start
To train the models, please first go to the `../data` folder and download the pre-processed SAPIEN dataset for Where2Act. 

To test over the pretrained models, please go to the `logs` folder and download the pretrained checkpoints.

Please fill in [this form](https://docs.google.com/forms/d/e/1FAIpQLSegEvIM22Ta44MrKM5d-guRE4aDR5K77ZQoInLWEyib-aeCFw/viewform?usp=sf_link) to download all resources.

## Dependencies
This code has been tested on Ubuntu 18.04 with Cuda 10.1, Python 3.6, and PyTorch 1.7.0.

First, install SAPIEN following

    pip install http://download.cs.stanford.edu/orion/where2act/where2act_sapien_wheels/sapien-0.8.0.dev0-cp36-cp36m-manylinux2014_x86_64.whl

For other Python versions, you can use one of the following

    pip install http://download.cs.stanford.edu/orion/where2act/where2act_sapien_wheels/sapien-0.8.0.dev0-cp35-cp35m-manylinux2014_x86_64.whl
    pip install http://download.cs.stanford.edu/orion/where2act/where2act_sapien_wheels/sapien-0.8.0.dev0-cp37-cp37m-manylinux2014_x86_64.whl
    pip install http://download.cs.stanford.edu/orion/where2act/where2act_sapien_wheels/sapien-0.8.0.dev0-cp38-cp38-manylinux2014_x86_64.whl

Please do not use the default `pip install sapien` as SAPIEN is still being actively developed and updated.

Then, if you want to run the 3D experiment, this depends on PointNet++.

    git clone --recursive https://github.com/erikwijmans/Pointnet2_PyTorch
    cd Pointnet2_PyTorch
    # [IMPORTANT] comment these two lines of code:
    #   https://github.com/erikwijmans/Pointnet2_PyTorch/blob/master/pointnet2_ops_lib/pointnet2_ops/_ext-src/src/sampling_gpu.cu#L100-L101
    pip install -r requirements.txt
    pip install -e .

Finally, run the following to install other packages.
   
    # make sure you are at the repository root directory
    pip install -r requirements.txt

to install the other dependencies.

For visualization, please install blender v2.79 and put the executable in your environment path.
Also, the prediction result can be visualized using MeshLab or the *RenderShape* tool in [Thea](https://github.com/sidch/thea).

## Simulator
You can run the following command to test and visualize a random interation in the simulation environment.

    python collect_data.py 40147 StorageFurniture 0 pushing

Change the shape id to other ids for testing other shapes, 
and modify the primitive action type to any of the six supported types: *pushing, pushing-up, pushing-left, pulling, pulling-up, pulling-left*. 
Run `python collection_data.py --help` to understand the full input arguments. 

After you ran the code, you will find a record for this interaction trial under `./results/40147_StorageFurniture_0_pushing_0`, from where you can see the full log, 2D image, 3D depth and interaction outcome.
You can run the following command to replay the interaction.

    python replay_data.py results/40147_StorageFurniture_0_pushing_0/result.json

If you want to run on a headless server, simple put `xvfb-run -a ` before any code command that runs the SAPIEN simulator.
Install the `xvfb` tool on your server if not installed.

## Generate Offline Training Data
Before training the network, we need to collect a large set of interaction trials via random exploration, using the script `scripts/run_gen_offline_data.sh`.
By default, this file generates data for all categories under the *pushing* primitive action. 
You can modify the content of the above file to generate data for different settings.

Generating enough offline interaction trials is necessary for a successful learning, and it may require many CPU hours (e.g. 10,000 hrs or more) for the data collection.
So, this offline data collection script is designed for you to parallelize the data generation on different machines and many CPU cores, by setting the proper `--starting_epoch`, `--num_epochs`, `--out_fn` and `--num_processes` parameters.
After the data generation, you need to move all the data to the same folder and create one `data_tuple_list.txt` file merging all output data index files.
Check the parameters for more information.

    python gen_offline_data.py --help

In our experiments, we train one network per primitive action but across all shape categories.
The table below summarizes our default offline data generation epochs.
These numbers are picked empirically such that the offline positive data reaches 10K to start off successful training.
If you use a different setting, such as training per shape category (e.g. on Cabinet only), you might need less epochs of data to collect offline.

| Primitive Action Type  | Training Epochs |   Testing Epochs   |
| ------------- | ------------- |  ---------------- |  
|  pushing |  50 | 10  |  
|  pushing-up | 120  | 30  |  
|  pushing-left | 100  | 30  |  
|  pulling | 250  | 70  |  
|  pulling-up | 130  | 30  |  
|  pulling-left | 130  | 30  |  

We also collect offline additional 100 epochs of data that will be used as the online random exploration data for each experiment.
This data can be generated offline and loaded online during the training to save the training time.
So, during training, we only need to spend time on collecting the online adaptatively sampled data.

## 3D Experiment
To train the network, first train the Action Scoring Module (critic) only until convergence,

    bash scripts/run_train_3d_critic.sh

then, train the full model (please specify the pre-trained critic-only network checkpoint),

    bash scripts/run_train_3d.sh

To evaluate and visualize the results, run

    bash scripts/run_visu_critic_heatmap.sh 40147
    
to visualize the Action Scoring Module predictions (Fig. 4 in the main paper).
This script use a random viewpoint and a random interaction direction, 
so you can run multiple times to get different results.
The results are generated under `logs/[exp-folder]/[result-folder]/`.

Please use 

    bash scripts/run_visu_action_heatmap_proposals.sh 40147
    
to visualize the Actionability Module and Action Proposal Module results (Fig. 1, 5 in the main paper).
This script will generate a GIF for all proposed successful interaction orientations for a randomly sampled pixel for interaction, 
so you can run multiple times to get different results.
The results are generated under `logs/[exp-folder]/[result-folder]/`.

