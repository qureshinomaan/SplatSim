import os

def launch_eval_slurm(work_dir):
    work_dir = os.path.abspath(work_dir)
    str = f"""#!/usr/bin/env bash
#SBATCH --partition=Main
#SBATCH --cpus-per-task=6
#SBATCH --gres=gpu:1
#SBATCH --qos=low
#SBATCH -o {work_dir}/slurm_eval.out # STDOUT
#SBATCH -e {work_dir}/slurm_eval.err # STDERR
srun hostname
ls
python RLAfford/dagger/mp_eval_launcher.py {work_dir}
    """
    # Save to file
    with open(f"{work_dir}/slurm_eval.sh", "w") as f:
        f.write(str)

    # Launch
    os.system(f"sbatch {work_dir}/slurm_eval.sh") # Run in background