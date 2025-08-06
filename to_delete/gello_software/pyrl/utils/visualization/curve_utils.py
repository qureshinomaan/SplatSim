"""
Create beautiful curves that can be used in the paper which matplotlib and seaborn
"""
import numpy as np, os, glob, pathlib
from scipy.ndimage import gaussian_filter1d
from scipy import interpolate
from collections import deque
from pyrl.utils.data import is_num


def interpolate_curve(x, y, x_min=None, x_max=None, x_int=1000):
    masks, mask = ([],)
    if x_min is None:
        masks.apppend(x > x_min)

    if x_max is None:
        masks.apppend(x < x_max)

    if len(masks) >= 2:
        mask = np.logical_and(*masks[:2])
        for i in range(2, len(masks)):
            mask = np.logical_and(mask, masks[i])
    else:
        mask = masks[0]

    # mask = np.logical_and(*mask)
    # mask = np.logical_and(x > limitx[0], x < limitx[1])
    # mask = np.logical_and(x > limitx[0], x < limitx[1])
    # x = x[mask]
    # y = y[mask]

    x_min = x_min // x_int * x_int
    x_max = x_max // x_int * x_int

    # xmin = limitx[0] // plt_intv * plt_intv
    # xmax = limitx[1] // plt_intv * plt_intv

    x_int = np.arange(x_min, x_max, x_int)
    f = interpolate.interp1d(x, y, kind="linear", bounds_error=False, fill_value=(y[0], y[-1]))
    y_int = f(x_int)
    return [x_int, y_int]


def interpolate_between_curves(y1, y2, c):
    assert len(y1) == len(y2) and 0 <= c and c <= 1
    return c * y1 + (1 - c) * y2


def smooth_curve(x, y, smooth_type="gaussian", smooth=20):
    if smooth_type == "gaussian":
        return x, gaussian_filter1d(y, sigma=smooth, mode="nearest")
    else:
        smooth = int(smooth // 2)
        x = x[smooth:]
        q = deque(maxlen=smooth * 2 + 1)  # [i - smooth, i + smooth]
        for i in range(smooth * 2):
            q.append(y[min(i, len(y) - 1)])
        ret = []
        for i in range(smooth, len(y)):
            q.append(y[min(i + smooth, len(y) - 1)])
            # print(2, len(q), type(q))
            ret.append(np.mean(list(q)))

        return x, np.array(ret)


def compute_std(xs, ys, use_std_err=False, target_x=None, interpolate_kind="linear", x_tolerance=1e4, return_std=True):
    if is_num(xs[0]):
        res = compute_std([xs], [ys], use_std_err=use_std_err, target_x=target_x, interpolate_kind=interpolate_kind)
        res = [_[0] for _ in res]
        return res

    assert len(xs) == len(ys)
    xs = [np.array(_) for _ in xs]
    ys = [np.array(_) for _ in ys]
    len_x = np.array([len(_) for _ in xs])
    len_y = np.array([len(_) for _ in ys])
    assert (len_x == len_y).all(), f"{len_x}, {len_y}"

    longest_one = np.argmax([_[-1] for _ in xs])
    x = xs[longest_one] if target_x is None else target_x

    yf = [interpolate.interp1d(xi, yi, kind=interpolate_kind, bounds_error=False, fill_value=(yi[0], yi[-1])) for xi, yi in zip(xs, ys)]
    ys_interpolated = [yf_i(x) for yf_i in yf]

    ys, stds = [], []
    for i, xi in enumerate(x):
        ys_i = [ys_interpolated[j][i] for j in range(len(xs)) if xi <= xs[j][-1] + x_tolerance]
        # print(ys_i, xi)

        if len(ys_i) > 1:
            ys.append(np.mean(ys_i))
            stds.append(np.std(ys_i))
        else:
            x = x[:i]
            break
    ys, stds = np.array(ys), np.array(stds)

    if return_std:
        if use_std_err:
            stds /= np.sqrt(len(xs))

        return x, ys, stds
    else:
        return x, ys
