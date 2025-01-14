import os.path as osp, shutil
import matplotlib.pyplot as plt
from .base_logger import BaseLogger
from ..data import is_num
from .builder import EXP_LOGGER


@EXP_LOGGER.register_module(name="tensorboard")
@EXP_LOGGER.register_module()
class TensorboardLogger(BaseLogger):
    def __init__(self, work_dir: str, log_dir: str = None, clean_up=False, timestamp=None, **kwargs):
        super().__init__(work_dir, timestamp)
        from torch.utils.tensorboard import SummaryWriter

        if log_dir is None:
            log_dir = osp.join(work_dir, "tf_logs")
        if clean_up:
            shutil.rmtree(log_dir, ignore_errors=True)

        self.writer = SummaryWriter(log_dir)
        self.writer.add_text("work_dir", osp.abspath(work_dir), 0)
        # self.csv_writer = 

    def log(self, tags, n_iter, tag_name="train"):
        tags = self.get_loggable_tags(tags, tag_name=tag_name)
        self.log_to_csv(tags, n_iter)
        for tag, val in tags.items():
            if isinstance(val, str):
                self.writer.add_text(tag, val, n_iter)
            elif is_num(val) or val.size == 1:
                self.writer.add_scalar(tag, val, n_iter)
            else:
                if val.ndim == 2:
                    cmap = plt.get_cmap("jet")
                    val = cmap(val)[..., :3]
                assert val.ndim == 3, f"Image should have two dimension! You provide: {tag, val.shape}!"
                self.writer.add_image(tag, val, n_iter, dataformats="HWC")

    def close(self):
        self.writer.close()
