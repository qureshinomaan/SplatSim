import tarfile
import os
import os.path as osp
import zipfile, time
from ..meta import get_total_memory
from ..data import auto_pad_seq
from .serialization import dump, load
from .hash_utils import md5sum, check_md5sum


def extract_files(filenames, target_folders):
    auto_pad_seq(filenames, target_folders)
    for filename, target_folder in zip(filenames, target_folders):
        if filename.endswith(".zip"):
            opener, mode = zipfile.ZipFile, "r"
        elif filename.endswith(".tar.gz") or filename.endswith(".tgz"):
            opener, mode = tarfile.open, "r:gz"
        elif filename.endswith(".tar.bz2") or filename.endswith(".tbz"):
            opener, mode = tarfile.open, "r:bz2"
        else:
            raise ValueError(f"Could not extract `{target_folder}` as no appropriate extractor is found")
        opener(filenames)
        try:
            file = opener(filename, mode)
            file.extractall(target_folder)
            file.close()
        except Exception as e:
            print(f"Cannot extract file {filename}")
            print(e)


class MultiFile(object):
    def __init__(self, file_name, max_file_size, max):
        self.current_position = 0
        self.file_name = file_name
        self.max_file_size = max_file_size
        self.current_file = None
        self.open_next_file()

    @property
    def current_file_no(self):
        return self.current_position / self.max_file_size

    @property
    def current_file_size(self):
        return self.current_position % self.max_file_size

    @property
    def current_file_capacity(self):
        return self.max_file_size - self.current_file_size

    def open_next_file(self):
        file_name = "%s.%03d" % (self.file_name, self.current_file_no + 1)
        if self.current_file is not None:
            self.current_file.close()
        self.current_file = open(file_name, "wb")

    def tell(self):
        print("MultiFile::Tell -> %d" % self.current_position)
        return self.current_position

    def write(self, data):
        start, end = 0, len(data)
        print("MultiFile::Write (%d bytes)" % len(data))
        while start < end:
            current_block_size = min(end - start, self.current_file_capacity)
            self.current_file.write(data[start : start + current_block_size])
            print("* Wrote %d bytes." % current_block_size)
            start += current_block_size
            self.current_position += current_block_size
            if self.current_file_capacity == self.max_file_size:
                self.open_next_file()
            print("* Capacity = %d" % self.current_file_capacity)

    def flush(self):
        print("MultiFile::Flush")
        self.current_file.flush()


def zip_files(files):
    if isinstance(files, (list, tuple)) and len(files) == 1:
        files = files[0]
    mfo = MultiFile("splitzip.zip", 2**18)

    zf = zipfile.ZipFile(mfo, mode="w", compression=zipfile.ZIP_DEFLATED)
    for i in range(4):
        filename = "test%04d.txt" % i
        print("Adding file '%s'..." % filename)
        # zf.writestr(filename, get_random_data(2**17))


def split_by_volume(file_path, block_size=1024**3):
    file_size = os.path.getsize(file_path)
    folder, file_name = os.path.split(file_path)
    if block_size == -1 or file_size < block_size:
        return [
            file_path,
        ]

    fp = open(file_path, "rb")
    count = int((file_size + block_size - 1) // block_size)
    save_dir = osp.join(folder, file_name + "_split")
    # print(save_dir)
    # exit(0)
    if os.path.exists(save_dir):
        from shutil import rmtree

        rmtree(save_dir)
    os.mkdir(save_dir)

    info = []
    for i in range(count):
        name = f"{file_name}.split{i}"
        with open(osp.join(save_dir, name), "wb+") as f:
            f.write(fp.read(block_size))
        info.append([name, md5sum(osp.join(save_dir, name))])
    fp.close()
    dump(info, osp.join(save_dir, f"{file_name}.index.csv"))
    return save_dir


def concat_split_files(split_folder):
    folder, file_name = os.path.split(split_folder)
    file_path = osp.join(folder, file_name[: -len("_split")])
    index = load(osp.join(split_folder, f'{file_name[:-len("_split")]}.index.csv'))
    fp = open(file_path, "wb")
    for i in range(len(index)):
        name, md5 = index[i]
        file_i = osp.join(split_folder, name)
        print(file_i, name, md5)
        assert check_md5sum(file_i, md5)
        with open(file_i, "rb") as f:
            fp.write(f.read())
    fp.close()
