import numpy as np, base64
from .dict_array import GDict
from .array_ops import encode_np, decode_np
from .converter import as_dtype
from .type_utils import is_np_arr, get_dtype, is_dict, is_not_null, is_null, is_seq_of
from gello.data_utils.meta import Config, merge_a_to_b


def float_to_int(data, vrange=[0.0, 1.0], res=None, dtype="uint8"):
    data_dtype = get_dtype(data)
    if "int" in data_dtype:
        return as_dtype(data, dtype) if data_dtype != dtype else data
    assert data_dtype.startswith("float"), f"{type(data), data}"
    min_v = np.iinfo(getattr(np, dtype)).min
    max_v = np.iinfo(getattr(np, dtype)).max
    if is_not_null(vrange):
        assert vrange[0] < vrange[1] and is_null(res)
        data = (np.clip(data, a_min=vrange[0], a_max=vrange[1]) - vrange[0]) / (vrange[1] - vrange[0])  # Normalize value to [0, 1]
        data = data * max_v + (1 - data) * min_v
    else:
        assert is_not_null(res)
        data = data / res

    data = as_dtype(np.clip(data, a_min=min_v, a_max=max_v), dtype)
    return data


def int_to_float(data, vrange=[0.0, 1.0], res=None, *dtype):
    data_dtype = get_dtype(data)
    if data_dtype == "object":
        assert data.shape == (1,)
        data = data[0]
    elif data_dtype.startswith("float"):
        return as_dtype(data, dtype) if data_dtype != dtype else data

    data_dtype = get_dtype(data)

    assert data_dtype.startswith("int") or data_dtype.startswith("uint"), f"{data_dtype}"
    min_v = np.float32(np.iinfo(getattr(np, data_dtype)).min)
    max_v = np.float32(np.iinfo(getattr(np, data_dtype)).max)
    if is_not_null(vrange):
        assert vrange[0] < vrange[1] and is_null(res)
        data = (data - min_v) / (max_v - min_v)  # [0, 1]
        data = data * np.float32(vrange[1]) + (1 - data) * np.float32(vrange[0])
    else:
        assert is_not_null(res)
        res = np.float32(res)
        data = data * res
    return as_dtype(data, "float32")


def f64_to_f32(item):
    """
    Convert all float64 data to float32
    """
    from .type_utils import get_dtype
    from .converter import as_dtype

    sign = get_dtype(item) in ["float64", "double"]
    return as_dtype(item, "float32") if sign else item


def to_f32(item):
    return as_dtype(item, "float32")


def to_f16(item):
    return as_dtype(item, "float16")

class DataCoder:
    """
    To reduced the filesize when storing data for deep learning, we need to first compreess the data.
    If the data cannot be represented as a numpy array, we can encode them into a binary string and store into hdf5.
    """

    ENCODE_SETTINGS = {
        "maniskill-rgbd": {
            # 'obs/rgbd/xyz': ,
            "obs/rgbd/rgb": "encode_rgb_png",
            "obs/rgbd/depth": "encode_depth_png",
            "obs/rgbd/seg": ("encode_seg_mask", 3),
            # 'obs/rgbd/seg': 'encode_seg_mask',
        },
        "pcd-variable": {
            "inputs/xyz": "encode_np",
            "inputs/rgb": "encode_np",
            "labels": "encode_np",
            "vote_xyz": "encode_np",
            "vote_center": "encode_np",
        },
        "pcd": {
            "vote_center": "encode_np",
        },
    }

    COMPRESS_SETTINGS = {
        "maniskill-rgbd": {
            "obs/rgbd/rgb": ("np_compress", [0.0, 1.0], None, "uint8"),
            "obs/rgbd/depth": ("np_compress", [0.0, 1.0], None, "uint16"),
        },
        "pcd": {
            "inputs/xyz": ("np_compress", None, 1e-3, "int16"),
            "inputs/rgb": ("np_compress", [0.0, 1.0], None, "uint8"),
            "xyz": ("np_compress", None, 1e-3, "int16"),
            "rgb": ("np_compress", [0.0, 1.0], None, "uint8"),
            "vote_xyz": ("np_compress", None, 1e-3, "int16"),
            "vote_center": ("np_compress", None, 1e-3, "int16"),
        },
    }

    def __init__(self, mode=None, encode_cfg=None, compress_cfg=None, var_len_item=False):
        self.mode = mode
        self.var_len_item = var_len_item
        encode_cfg = merge_a_to_b(encode_cfg, self.ENCODE_SETTINGS.get(mode, None))
        compress_cfg = merge_a_to_b(compress_cfg, self.COMPRESS_SETTINGS.get(mode, None))
        pop_null = lambda _: {key: value for key, value in _.items() if is_not_null(value)}
        self.encode_cfg = None if is_null(encode_cfg) else pop_null(encode_cfg)
        self.compress_cfg = None if is_null(compress_cfg) else pop_null(compress_cfg)

    # Encode functions [For single item]
    def uint8_png(self, arr, encode):
        from ..image import imencode, imdecode

        if encode:
            num_image = arr.shape[-1] // 3
            assert num_image * 3 == arr.shape[-1]
            arr = np.split(arr, num_image, axis=-1)
            arr = [imencode(_) for _ in arr]
        else:
            arr = np.concatenate([imdecode(_[0]) for _ in arr], axis=-1)
        return arr

    def uint16_png(self, arr, encode):
        from ..image import imencode, imdecode

        if encode:
            num_image = arr.shape[-1]
            arr = np.split(arr, num_image, axis=-1)
            arr = [imencode(_) for _ in arr]
        else:
            arr = np.concatenate([imdecode(_[0]) for _ in arr], axis=-1)
        return arr

    def seg_png(self, arr, encode, num_images=None):
        from ..image import imencode, imdecode

        if encode:
            arr = np.split(arr, num_images, axis=-1)
            assert arr[0].shape[-1] <= 8
            arr = [np.packbits(np.concatenate([_, np.ones_like(_[..., :1])], axis=-1), axis=-1, bitorder="little") for _ in arr]
            arr = [imencode(_) for _ in arr]
        else:
            arr = [imdecode(_[0])[..., None] for _ in arr]
            num_segs = int(arr[0][0, 0, 0]).bit_length() - 1
            arr = np.concatenate([np.unpackbits(_, axis=-1, count=num_segs, bitorder="little") for _ in arr], axis=-1).astype(np.bool_)
        return arr

    def encode_np(self, arr, encode, *args):
        if encode:
            return encode_np(arr, *args)
        else:
            return decode_np(arr, *args)

    # Compress functions [For batched inputs]
    def np_compress(self, arr, encode, *args):
        if encode:
            return float_to_int(arr, *args)
        else:
            return int_to_float(arr, *args)

    @GDict.wrapper(class_method=True)
    def _apply(self, data, cfg, encode=False):
        if encode:
            data = data.f64_to_f32()
        if is_null(cfg):
            return data
        for key, item in cfg.items():
            if isinstance(item, (list, tuple)):
                args = item[1:]
                item = item[0]
            else:
                args = []
            if key in data:
                data[key] = getattr(self, item)(data[key], encode, *args)
        return data

    def encode(self, data):
        return self._apply(data, self.encode_cfg, True)

    def decode(self, data):
        return self._apply(data, self.encode_cfg, False)

    def compress(self, data):
        return self._apply(data, self.compress_cfg, True)

    def decompress(self, data):
        return self._apply(data, self.compress_cfg, False)
