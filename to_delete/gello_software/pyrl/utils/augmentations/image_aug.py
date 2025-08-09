import math, random, numpy as np
import torch, torch.nn.functional as F
import torchvision.transforms.functional_tensor as F_t
import torchvision.transforms.functional as F_v

from numbers import Number
from .builder import AUGMENTATIONS, BaseAugmentation
from pyrl.utils.data import GDict


@AUGMENTATIONS.register_module()
class RandomCrop(BaseAugmentation):
    """Crop the given Image at a random location.

    padding (int or sequence, optional): Optional padding on each border
    of the image. If a sequence of length 4 is provided, it is used to
    pad left, top, right, bottom borders respectively.  If a sequence
    of length 2 is provided, it is used to pad left/right, top/bottom
    borders, respectively. Default: None, which means no padding.

    Warning: size is (h, w) however padding is defined as [w, h, w, h] or [w, h]
    """

    def __init__(
        self,
        main_key="rgb",
        req_keys=["rgb"],
        size=None,
        padding=None,
        pad_if_needed=False,
        pad_val=0,
        padding_mode="constant",
        use_kornia=False,
    ):
        super().__init__(main_key, req_keys)
        assert padding_mode in ["constant", "reflect", "edge", "symmetric"]

        self.size = (size, size) if isinstance(size, Number) else np.array(size, dtype=np.int64)
        assert len(self.size) == 2

        self.padding = padding
        self.pad_val = pad_val
        self.padding_mode = padding_mode

        self.use_kornia = use_kornia
        self.pad_if_needed = pad_if_needed

        if use_kornia:
            assert len(req_keys) == 1, "kornia only support single key, e.g., rgb or depth"
            from kornia.augmentation import RandomCrop as Kornia_RandomCrop

            self.img_aug = Kornia_RandomCrop(
                tuple(self.size),
                padding=padding,
                padding_mode=padding_mode if padding_mode != "edge" else "replicate",
                fill=pad_val,
                pad_if_needed=pad_if_needed,
                same_on_batch=False,
            )

    def process_single(self, data, key):
        assert data.ndim in [3, 4, 5], "[C, H, W], [B, C, H, W] or [B, N, C, H, W]"
        data_shape, device, data_dtype, [th, tw] = data.shape, data.device, data.dtype, self.size
        batch_shape, num_channels, [h, w] = data_shape[:-3], data_shape[-3], data_shape[-2:]

        if self.use_kornia:
            data = self.img_aug(data.reshape((-1,) + tuple(data.shape[-3:])).float()).reshape(data.shape).to(data_dtype)
            return data

        if self.padding is not None:
            data = F_v.pad(data, self.padding, self.pad_val, self.padding_mode)

        # pad the image if the image is smaller than required size!
        if self.pad_if_needed:
            dh, dw = max(h - th, 0), max(w - tw, 0)
            if dh > 0 or dw > 0:
                data = F_v.pad(data, (dw, dh), self.pad_val, self.padding_mode)

        [h, w] = data.shape[-2:]
        if h == th and w == tw:
            return data

        if self.infos is None:
            i = torch.randint(0, h - th + 1, size=batch_shape, device=device)[..., None, None, None] + torch.arange(th, device=device)[..., None]
            j = torch.randint(0, w - tw + 1, size=batch_shape, device=device)[..., None, None, None] + torch.arange(tw, device=device)
            i = i.repeat_interleave(w, -1)
            j = j.repeat_interleave(th, -2)
            self.infos = [i, j]
        i, j = GDict(self.infos).repeat(num_channels, -3)
        data = torch.gather(data, -2, i)
        data = torch.gather(data, -1, j)
        return data

    def __repr__(self):
        return self.__class__.__name__ + f"(size={self.size}, padding={self.padding})"

def img_obs_wrapper(func):
    def wrapper(self, obs):
        if isinstance(obs, dict):
            return func(self, obs)
        else:
            raise NotImplementedError(f"Type of data is {type(obs)}")

    return wrapper


def torch_imsign(rgb, prob, independent=False):
    # rgb: [..., 3 * K, H, W]
    num_images = rgb.shape[-3] // 3  # Value K
    batch_shape = list(rgb.shape[:-3])
    if independent:
        sign = torch.rand(*(batch_shape + [num_images, 1, 1]), device=rgb.device) <= prob
    else:
        sign = torch.rand(*(batch_shape + [1, 1, 1]), device=rgb.device) <= prob
        sign = sign.repeat_interleave(num_images, -3)
    return sign


# Augmentations


@AUGMENTATIONS.register_module()
class ToChannelFirst:
    """
    Swith channels of features to make sure the feature channel is the last one
    """

    @img_obs_wrapper
    def __call__(self, rgbd):
        for key in rgbd.keys():
            order = list(range(rgbd[key].ndim))
            order = (
                order[:-3]
                + [
                    order[-1],
                ]
                + order[-3:-1]
            )
            rgbd[key] = rgbd[key].permute(*order)
        return rgbd

    def __repr__(self):
        return self.__class__.__name__ + f"feature channel={self.channels}"


@AUGMENTATIONS.register_module()
class ToChannelLast:
    """
    Swith channels of features to make sure the feature channel is the last one
    """

    @img_obs_wrapper
    def __call__(self, rgbd):
        for key in rgbd.keys():
            order = list(range(rgbd[key].ndim))
            order = (
                order[:-3]
                + order[-2:]
                + [
                    order[-3],
                ]
            )
            rgbd[key] = rgbd[key].permute(*order)
        return rgbd

    def __repr__(self):
        return self.__class__.__name__ + f"feature channel={self.channels}"
