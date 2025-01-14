import os.path as osp, shutil, os, numpy as np
from .serialization import load, dump


class LMDBFile:
    def __init__(self, db_path, readonly=True, lock=True, all_async=False, readahead=False, replace=True, map_size=2 * (1024**4)):
        replace = replace and not readonly
        if replace:
            shutil.rmtree(db_path, ignore_errors=True)
            os.makedirs(db_path, exist_ok=True)
        import lmdb

        self.env = lmdb.open(
            db_path,
            subdir=osp.isdir(db_path),
            map_size=map_size,
            readonly=readonly,
            metasync=all_async,
            sync=all_async,
            create=True,
            readahead=readahead,
            writemap=False,
            meminit=False,
            map_async=all_async,
            lock=lock,
        )

        self.readonly = readonly
        self.modified = False
        self.init_with_info = False
        self.writer = None
        self.reader = None
        self.length = len(self)

        if self.readonly:
            self.build_reader()
        else:
            self.build_writer()

    def __len__(self):
        return self.env.stat()["entries"]

    def close(self):
        if self.writer is not None:
            self.commit()
        self.env.sync()
        self.env.close()

    def build_writer(self):
        self.writer = self.env.begin(write=True)

    def build_reader(self):
        del self.reader
        self.reader = self.env.begin(write=False)

    def commit(self):
        self.writer.commit()
        self.writer = self.env.begin(write=True)

    def write_item(self, item):
        key = str(self.length)
        self.writer.put(key.encode(), dump(item, file_format="pkl"))
        self.length += 1
        if self.length % 100 == 0:
            self.commit()
