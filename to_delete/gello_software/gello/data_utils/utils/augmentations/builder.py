from collections.abc import Sequence

from pyrl.utils.data import is_null, is_not_null, GDict, DictArray
from pyrl.utils.meta import Registry, build_from_cfg


AUGMENTATIONS = Registry("data augmentation")
# The augmentation should happens in torch which supports aysnchronized execution!!


class DataAugmentations:
    """Compose a data pipeline with a sequence of transforms.
    Args:
        transforms (list[dict | callable]):
            Either config dicts of transforms or transform objects.
    """

    def __init__(self, transforms):
        assert isinstance(transforms, Sequence)
        self.transforms = []
        for transform in transforms:
            if isinstance(transform, dict):
                transform = build_from_cfg(transform, AUGMENTATIONS)
                self.transforms.append(transform)
            elif callable(transform):
                self.transforms.append(transform)
            else:
                raise TypeError(f"transform must be callable or a dict, but got {type(transform)}")

    # @GDict.wrapper(class_method=True)
    def __call__(self, data, begin_index=0):
        for i, t in enumerate(self.transforms[begin_index:]):
            data = t(data)
            if data is None:
                return None
        return data

    def __getitem__(self, key):
        return self.transforms[key]

    def __repr__(self):
        format_string = self.__class__.__name__ + "("
        for t in self.transforms:
            format_string += f"\n    {t}"
        format_string += "\n)"
        return format_string


@AUGMENTATIONS.register_module()
class BaseAugmentation:
    def __init__(self, main_key=None, req_keys=None):
        self.main_key = main_key
        self.req_keys = req_keys or [main_key]
        assert main_key in req_keys, f"{main_key}, {req_keys} do not satisfy the requirement!"

        self.infos = None
        self.full_data = None

    def process_single(self, item, key=None):
        """
        Apply the transformation for the single key
        """
        return item

    def process(self, data):
        """
        Apply the same transformation for different components in the data
        """
        data = GDict(data)
        self.infos, self.full_data = None, data
        if is_null(self.req_keys):
            data.memory = self.process_single(data)
        else:
            for key in self.req_keys:
                if key in data:
                    data[key] = self.process_single(data[key], key)
        return data.memory

    def _is_var_list(self, data):
        if is_not_null(self.main_key):
            data = data[self.main_key]
        return isinstance(data, (list, tuple))

    def __call__(self, data):
        data = GDict(data)  # Shallow copy
        if is_not_null(self.main_key):
            assert self.main_key in data, f"{self.main_key}, {GDict(data).shape}"
        if self._is_var_list(data):
            batch_size = len(data[self.main_key])
            for i in range(batch_size):
                data_i = data.take_list(i).unsqueeze(0)
                self.process(data_i)
                data.assign_list(i, data_i.squeeze(0))
        else:
            data = self.process(data)
        return data


def build_data_augmentations(cfg, default_args=None):
    if cfg is None:
        return None
    if not isinstance(cfg, (list, tuple)):
        cfg = [cfg]
    return DataAugmentations(cfg)
