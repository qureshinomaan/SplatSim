from mani_skill_learn.utils.fileio import check_md5sum, load
import os.path as osp


if __name__ == "__main__":
    __this__folder = osp.dirname(__file__)
    all_files = load(osp.join(__this__folder, 'all_data_files.csv'))
    for file_path, md5 in all_files:
        file_path = osp.join(__this__folder, '../', file_path)
        if not osp.exists(file_path):
            print(f"{osp.abspath(file_path)} does not exist!")
            continue
        if not check_md5sum(file_path, md5):
            print(f"{osp.abspath(file_path)} is not correctly downloaded!")
