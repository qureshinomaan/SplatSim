import os
from glob import glob
import click
from natsort import natsorted
import numpy as np
import csv

def read_csv(csv_file):
    # Open the CSV file
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        next(reader)
        names , successes= [], []
        for row in reader:
            # Extract the columns
            names.append(row[0])
            successes.append(float(row[1]))
    return names, successes

def aggregate_results(folder):
    if not os.path.exists(folder):
        print(f'Folder "{folder}" does not exist')
        return
    csv_files = glob(os.path.join(folder, 'result_*.csv'))
    csv_files = natsorted(csv_files)
    save_file = os.path.join(folder, 'summary.csv')

    all_successes = []
    for csv_file in csv_files:
        names, successes = read_csv(csv_file)
        all_successes.append(successes)
    successes = np.array(all_successes).mean(axis=0)

    with open(save_file, 'w') as f:
        f.write('policy_name,success\n')
        for name, success in zip(names, successes):
            f.write(f'{name},{success}\n')


# def aggregate_results(x):
@click.command()
@click.argument('exp_folder', type=click.Path(exists=True))
@click.option('--n_assets', default=8, help='Number of assets per chunk')
@click.option('--n_eval', default=5, help='Number of rollouts to evaluate')
def main(exp_folder, n_assets, n_eval):
    all_exp_video_files = glob(os.path.join(exp_folder, '**', 'videos'), recursive=True)
    all_exp_folders = natsorted([os.path.dirname(video_folder) for video_folder in all_exp_video_files])

    for exp_folder in all_exp_folders:
        if 'drawer' in exp_folder:
            total_assets = 12
        elif 'door' in exp_folder:
            total_assets = 8
        else:
            print(f'Exp folder "{exp_folder}" not run as it is not a drawer or door experiment')

        num_chunks = round(total_assets / n_assets)
        for i in range(0, num_chunks):
            cmd = f'python RLAfford/dagger/eval.py {exp_folder} --num_chunks={num_chunks} --chunk_id={i} --n_eval={n_eval}'
            print(cmd)
            os.system(cmd)
        eval_folder = os.path.join(exp_folder, 'eval')
        aggregate_results(eval_folder)

if __name__ == '__main__':
    main()