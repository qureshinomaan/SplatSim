import torch, numpy as np
from pyrl.utils.data import to_torch


def get_o3d_corners(device="numpy"):
    """Get 8 corners of a cuboid. (The order follows OrientedBoundingBox in open3d)

      (y)
      2 -------- 7
     /|         /|
    5 -------- 4 .
    | |        | |
    . 0 -------- 1 (x)
    |/         |/
    3 -------- 6
    (z)
    """
    corners = (
        np.array(
            [
                [0, 0, 0],
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1],
                [1, 1, 1],
                [0, 1, 1],
                [1, 0, 1],
                [1, 1, 0],
            ],
            np.float32,
        )
        - 0.5
    )
    if device != "numpy":
        corners = to_torch(corners, device=device)
    return corners


def get_pytorch3d_corners(device="numpy"):
    corners = (
        np.array(
            [
                [0, 0, 0],
                [1, 0, 0],
                [1, 1, 0],
                [0, 1, 0],
                [0, 0, 1],
                [1, 0, 1],
                [1, 1, 1],
                [0, 1, 1],
            ],
            dtype=np.float32,
        )
        - 0.5
    )
    if device != "numpy":
        corners = to_torch(corners, device=device)
    return corners


def corner_emd_loss(pred_rot, gt_rot, gt_size=None, proj_axis=None, reduction="batch", corners_mode="pytorch3d"):
    """Corner loss (Earth Mover Distance, given match). A proxy loss for rotation.

    Args:
        pred_rot (torch.Tensor): [B, 3, 3]
        gt_rot (torch.Tensor): [B, 3, 3]
        gt_size (optional, torch.Tensor): [B, 3]
        proj_axis (optional, torch.Tensor): [B, 3].
            The axis to project, especially for infinite symmetry order.
        reduction (str): none, batch

    Returns:
        torch.Tensor: scalar
    """
    assert corners_mode in ["pytorch3d", "open3d"]
    if gt_size is None:
        gt_size = torch.ones_like(pred_rot[..., 3])
    if corners_mode == "pytorch3d":
        corners = get_pytorch3d_corners(device=pred_rot.device)  # [8, 3]
    else:
        corners = get_o3d_corners(device=pred_rot.device)  # [8, 3]

    if proj_axis is None:
        corners = corners.unsqueeze(0).expand(pred_rot.size(0), 8, 3)
    else:
        # [B, 8, 3]
        corners = torch.einsum("bi,mi->bm", proj_axis, corners).unsqueeze(-1) * proj_axis.unsqueeze(1)

    pred_pts = torch.einsum("bij,bmj->bmi", pred_rot, corners)
    pred_pts = pred_pts * gt_size.unsqueeze(1)
    gt_pts = torch.einsum("bij,bmj->bmi", gt_rot, corners)
    gt_pts = gt_pts * gt_size.unsqueeze(1)

    loss_emd = torch.norm(pred_pts - gt_pts, dim=-1)
    if reduction == "none":
        loss_emd = loss_emd
    elif reduction == "batch":
        loss_emd = loss_emd.mean(-1)
    else:
        loss_emd = loss_emd.mean()
    return loss_emd
