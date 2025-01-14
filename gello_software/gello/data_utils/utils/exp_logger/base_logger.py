import numbers, numpy as np
import csv
import os.path as osp
from pyrl.utils.meta import get_logger


class BaseLogger:
    def __init__(self, work_dir, timestamp) -> None:
        self.work_dir = work_dir
        self.logging_keys = []
        self.csv_file = open(osp.join(work_dir, f"{timestamp}-exp_logging.csv"), "w")
        self.logger = get_logger

    def get_lr_tags(self, runner):
        tags = {}
        lrs = runner.current_lr()
        if isinstance(lrs, dict):
            for name, value in lrs.items():
                tags[f"learning_rate/{name}"] = value[0]
        else:
            tags["learning_rate"] = lrs[0]
        return tags

    def log_to_csv(self, loggable_tags, n_iter):
        def clean_up_tags(tags, n_iter):
            from pyrl.utils.data import is_np_arr

            clean_tags = dict()
            mismatch = []
            for key, val in tags.items():
                if key in self.logging_keys:
                    if is_np_arr(val) and len(val) == 1:
                        clean_tags[key] = val[0]
                    else:
                        clean_tags[key] = val
                else:
                    mismatch.append(key)
            clean_tags["n_iter"] = n_iter
            return clean_tags, mismatch

        if len(self.logging_keys) == 0:
            self.logging_keys.extend(["n_iter"] + list(loggable_tags.keys()))
            csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.logging_keys)
            csv_writer.writeheader()
        cleaned_tags, mismatch = clean_up_tags(loggable_tags, n_iter)
        csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.logging_keys)
        csv_writer.writerow(cleaned_tags)
        self.csv_file.flush()

    def get_momentum_tags(self, runner):
        tags = {}
        momentums = runner.current_momentum()
        if isinstance(momentums, dict):
            for name, value in momentums.items():
                tags[f"momentum/{name}"] = value[0]
        else:
            tags["momentum"] = momentums[0]

        return tags

    @staticmethod
    def is_scalar(val, include_np=True, include_torch=True):
        if isinstance(val, numbers.Number):
            return True
        elif include_np and isinstance(val, np.ndarray) and val.ndim == 0:
            return True
        else:
            import torch

            if include_torch and isinstance(val, torch.Tensor) and len(val) == 1:
                return True
            else:
                return False

    def get_loggable_tags(self, output, allow_scalar=True, allow_text=False, tags_to_skip=("time", "data_time"), add_mode=True, tag_name="train"):
        tags = {}
        for tag, val in output.items():
            if tag in tags_to_skip:
                continue
            if self.is_scalar(val) and not allow_scalar:
                continue
            if isinstance(val, str) and not allow_text:
                continue
            if add_mode and "/" not in tag:
                tag = f"{tag_name}/{tag}"
            tags[tag] = val
        return tags
