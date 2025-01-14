from enum import Enum
import numpy as np


class Color(Enum):
    red = (0, 0, 255)
    green = (0, 255, 0)
    blue = (255, 0, 0)
    cyan = (255, 255, 0)
    yellow = (0, 255, 255)
    magenta = (255, 0, 255)
    white = (255, 255, 255)
    black = (0, 0, 0)


def color_val(color):
    """Convert various input to color tuples.
    Args:
        color (:obj:`Color`/str/tuple/int/ndarray): Color inputs
    Returns:
        tuple[int]: A tuple of 3 integers indicating BGR channels.
    """
    if isinstance(color, str):
        return Color[color].value
    elif isinstance(color, Color):
        return color.value
    elif isinstance(color, tuple):
        assert len(color) == 3
        for channel in color:
            assert 0 <= channel <= 255
        return color
    elif isinstance(color, int):
        assert 0 <= color <= 255
        return color, color, color
    elif isinstance(color, np.ndarray):
        assert color.ndim == 1 and color.size == 3
        assert np.all((color >= 0) & (color <= 255))
        color = color.astype(np.uint8)
        return tuple(color)
    else:
        raise TypeError(f"Invalid type for color: {type(color)}")


def get_colormap(N=256, normalized=False):
    def bitget(byteval, idx):
        return int((byteval & (1 << idx)) != 0)

    dtype = "float32" if normalized else "uint8"
    cmap = np.zeros((N, 3), dtype=dtype)
    for i in range(N):
        r = g = b = 0
        c = i
        for j in range(8):
            r = r | (bitget(c, 0) << 7 - j)
            g = g | (bitget(c, 1) << 7 - j)
            b = b | (bitget(c, 2) << 7 - j)
            c = c >> 3
        cmap[i] = [r, g, b]
    cmap = cmap / 255 if normalized else cmap
    return cmap


PALETTE_C = get_colormap(256, False)


NYU40_COLOR_PALETTE = [
    (0, 0, 0),
    (174, 199, 232),  # wall
    (152, 223, 138),  # floor
    (31, 119, 180),  # cabinet
    (255, 187, 120),  # bed
    (188, 189, 34),  # chair
    (140, 86, 75),  # sofa
    (255, 152, 150),  # table
    (214, 39, 40),  # door
    (197, 176, 213),  # window
    (148, 103, 189),  # bookshelf
    (196, 156, 148),  # picture
    (23, 190, 207),  # counter
    (178, 76, 76),
    (247, 182, 210),  # desk
    (66, 188, 102),
    (219, 219, 141),  # curtain
    (140, 57, 197),
    (202, 185, 52),
    (51, 176, 203),
    (200, 54, 131),
    (92, 193, 61),
    (78, 71, 183),
    (172, 114, 82),
    (255, 127, 14),  # refrigerator
    (91, 163, 138),
    (153, 98, 156),
    (140, 153, 101),
    (158, 218, 229),  # shower curtain
    (100, 125, 154),
    (178, 127, 135),
    (120, 185, 128),
    (146, 111, 194),
    (44, 160, 44),  # toilet
    (112, 128, 144),  # sink
    (96, 207, 209),
    (227, 119, 194),  # bathtub
    (213, 92, 176),
    (94, 106, 211),
    (82, 84, 163),  # otherfurn
    (100, 85, 144),
]

SCANNET_COLOR_PALETTE = [
    (174, 199, 232),  # wall
    (152, 223, 138),  # floor
    (31, 119, 180),  # cabinet
    (255, 187, 120),  # bed
    (188, 189, 34),  # chair
    (140, 86, 75),  # sofa
    (255, 152, 150),  # table
    (214, 39, 40),  # door
    (197, 176, 213),  # window
    (148, 103, 189),  # bookshelf
    (196, 156, 148),  # picture
    (23, 190, 207),  # counter
    (247, 182, 210),  # desk
    (219, 219, 141),  # curtain
    (255, 127, 14),  # refrigerator
    (158, 218, 229),  # shower curtain
    (44, 160, 44),  # toilet
    (112, 128, 144),  # sink
    (227, 119, 194),  # bathtub
    (82, 84, 163),  # otherfurn
    (105, 105, 105),  # Points do not need to consider
]

colormap_from_name = {
    "scannet": np.array(SCANNET_COLOR_PALETTE, dtype=np.uint8),
    "nyu40_raw": np.array(NYU40_COLOR_PALETTE, dtype=np.uint8),
    "nyu40": np.array(NYU40_COLOR_PALETTE[1:], dtype=np.uint8),
    None: PALETTE_C,
}


def label_to_color(label, colormap=None, noramlized=False):
    if isinstance(colormap, str) or colormap is None:
        colormap = colormap_from_name[colormap]
    assert colormap.ndim == 2 and colormap.shape[1] == 3
    rgb = np.slice(colormap, label, axis=0)
    if noramlized:
        rgb = (rgb / 255.0).astype(np.float32)
    return rgb


def color_to_label(rgb, colormap=None):
    if isinstance(colormap, str) or colormap is None:
        colormap = colormap_from_name[colormap]
    assert colormap.ndim == 2 and colormap.shape[1] == 3 and rgb.shape[-1] == 3
    extend_rgb = rgb[..., None, :]
    extend_shape = (1,) * len(rgb.shape[:-1]) + colormap.shape
    extend_colormap = colormap.reshape(*extend_shape)
    label = np.all(extend_rgb == extend_colormap, axis=-1).argmax(-1)
    return label.astype(np.uint8)
