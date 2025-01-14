import argparse, glob
from mani_skill_learn.utils.fileio import generate_chunked_h5_replay
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description='Split a list of H5 File into small pieces')
    # Configurations
    parser.add_argument('h5_files', help='The list of h5 files or one h5 file or a folder that contains all h5 files.')
    parser.add_argument('--name', help='the name of the dataset', type=str, required=True)
    parser.add_argument('--output-folder', help='the name of the output folder', type=str, required=True)
    parser.add_argument('--num-files', help='the number of output files', type=int, required=True)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if isinstance(args.h5_files, str):
        if args.h5_files.endswith(".h5"):
            args.h5_files = [args.h5_files]
        else:
            args.h5_files = [str(_) for _ in Path(args.h5_files).glob('*.h5')]
    else:
        assert isinstance(args.h5, (list, tuple))
    generate_chunked_h5_replay(args.h5_files, args.name, args.output_folder, args.num_files)
