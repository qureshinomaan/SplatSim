import numpy as np
from pyrl.utils.data import float_to_int, as_dtype, sample_and_pad, is_np
from pyrl.utils.meta import get_logger
from pyrl.version import __version__


def select_mask(obs, key, mask):
    if key in obs:
        obs[key] = obs[key][mask]


def pcd_filter_ground(pcd, eps=1e-3):
    return pcd["xyz"][..., 2] > eps


def pcd_filter_with_mask(obs, mask, env=None):
    # A;; modification happen in this function will be in-place.
    assert isinstance(obs, dict), f"{type(obs)}"
    if "xyz" in obs:
        # Raw point cloud
        for key in ["xyz", "rgb", "seg"]:
            select_mask(obs, key, mask)
    else:
        pcd_filter_with_mask(obs, mask, env)
        for key in ["inst_seg", "target_seg"]:
            select_mask(obs, key, mask)


def pcd_base(obs, n_points=1200, min_pts=50, fg_pts=800):
    mask = obs["xyz"][:, 2] > 1e-3
    for key in ["xyz", "rgb", "seg"]:
        select_mask(obs, key, mask)
    for key in ["inst_seg", "target_seg"]:
        select_mask(obs, key, mask)

    seg = obs["seg"]
    tot_pts = n_points
    # target_mask_pts = n_points // 3 * 2
    # min_pts = n_points // 24

    num_pts = seg.sum(0)
    base_num = np.minimum(num_pts, min_pts)

    remain_pts = num_pts - base_num
    tgt_pts = base_num + (fg_pts - base_num.sum()) * remain_pts // remain_pts.sum()
    back_pts = tot_pts - tgt_pts.sum()

    bk_seg = ~seg.any(-1, keepdims=True)
    seg_all = np.concatenate([seg, bk_seg], axis=-1)
    num_all = seg_all.sum(-1)
    tgt_pts = np.concatenate([tgt_pts, np.array([back_pts])], axis=-1)

    chosen_index = []
    for i in range(seg_all.shape[1]):
        if num_all[i] == 0:
            continue
        cur_seg = np.where(seg_all[:, i])[0]
        np.random.shuffle(cur_seg)
        shuffle_indices = cur_seg[: tgt_pts[i]]
        chosen_index.append(shuffle_indices)
    chosen_index = np.concatenate(chosen_index, axis=0)

    if len(chosen_index) < tot_pts:
        n, m = tot_pts // len(chosen_index), tot_pts % len(chosen_index)
        chosen_index = np.concatenate([chosen_index] * n + [chosen_index[:m]], axis=0)
    for key in ["xyz", "rgb", "seg"]:
        select_mask(obs, key, chosen_index)
    for key in ["inst_seg", "target_seg"]:
        select_mask(obs, key, chosen_index)
    return obs


"""
def pcd_target_object_only(obs, env=None, num=1024):
    # cabinet or chair
    rgb = obs["pointcloud"]["rgb"]
    xyz = obs["pointcloud"]["xyz"]
    seg = obs["pointcloud"]["seg"]

    useful_seg = seg[:, -2] if seg.shape[-1] == 3 else ~seg[:, 0]
    if useful_seg.astype(np.int).sum() == 0:
        useful_seg = seg[:, -1]
    mask = np.logical_and(xyz[:, 2] > 1e-4, useful_seg)
    if mask.astype(np.int).sum() > 0:
        rgb = rgb[mask]
        xyz = xyz[mask]
        seg = seg[mask]
    # assert xyz.shape[0] > 0, f"{mask.astype(np.int).sum(), (xyz[:, 2] > 1e-4).astype(np.int).sum()}"

    index = np.arange(xyz.shape[0])
    np.random.shuffle(index)
    if index.shape[0] > num:
        index = index[:num]
    else:
        num_repeat = num // index.shape[0]
        index = np.concatenate([index for i in range(num_repeat)])
        index = np.concatenate([index, index[: num - index.shape[0]]])
    xyz = xyz[index]
    rgb = rgb[index]
    seg = seg[index]
    obs["pointcloud"]["rgb"] = rgb.astype(np.float16)
    obs["pointcloud"]["seg"] = seg
    obs["pointcloud"]["xyz"] = xyz.astype(np.float16)
    return obs
"""


def pcd_uniform_downsample(obs, env=None, ground_eps=1e-3, num=1200):
    obs_mode = env.obs_mode
    assert obs_mode in ["pointcloud", "fused_pcd", "pointcloud_3d_ann"]  # For fused pcd and voxel downsampled pcd

    if ground_eps is not None:
        pcd_filter_with_mask(obs, pcd_filter_ground(obs, eps=ground_eps), env)
    pcd_filter_with_mask(obs, sample_and_pad(obs["xyz"].shape[0], num), env)
    return obs


def pcd_voxel_downsample(obs, env=None, ground_eps=1e-3, num=1200):
    obs_mode = env.obs_mode
    assert obs_mode in ["pointcloud", "pointcloud_3d_ann"]  # Voxel uniform downsample
    pcd = obs
    if ground_eps is not None:
        pcd_filter_with_mask(obs, pcd_filter_ground(pcd, eps=ground_eps), env)

    voxel_size = 0.02
    index = np.arange(pcd["xyz"].shape[0])
    np.random.shuffle(index)
    voxel_xyz = (pcd["xyz"][index] // voxel_size).astype(np.int64)
    voxel_xyz = voxel_xyz - voxel_xyz.min(0)

    max_xyz = voxel_xyz.max(0) + 1
    voxel_xyz = voxel_xyz[..., 0] + voxel_xyz[..., 1] * max_xyz[0] + voxel_xyz[..., 2] * max_xyz[0] * max_xyz[1]

    unique_index = np.unique(voxel_xyz, return_index=True)[1]
    index = index[unique_index]
    pcd_filter_with_mask(obs, index, env)
    pcd_uniform_downsample(obs, env, None, num)
    return obs
