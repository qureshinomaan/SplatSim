import torch, numpy as np
from functools import wraps

# from pyrl.utils.data.dict_array import DictArray
from .builder import AUGMENTATIONS, BaseAugmentation
from pyrl.utils.torch import batch_rot_with_axis
from pyrl.utils.meta import dict_of
from pyrl.utils.data import (
    is_seq_of,
    is_dict,
    is_null,
    first_dict_key,
    is_not_null,
    batch_perm,
    gather,
    is_torch,
    GDict,
    DictArray,
    to_gc,
    to_nc,
    einsum,
    broadcast_to,
)
from torchvision.transforms import ColorJitter


def transform_pcd(xyz, mat=None, trans=None, rot=None, scale=None, order="RST"):
    """
    xyz: [B, N, 3] # or [B, 3] or [B, N, 2] or [B, 2]
    mat: [B, 4, 4]
    trans: [B, 3] or [B, N, 3]
    rot: [B, 3, 3]
    scale: [B]
    """
    if xyz is None:
        return None
    assert xyz.ndim == 3 and xyz.shape[-1] == 3, "Only support 3D xyz recently!"
    assert len(order) <= 3
    if mat is not None:
        assert (
            trans is None and rot is None and scale is None and mat.ndim == 3 and mat.shape[-1] == mat.shape[-2] == 4
        ), "When you provide a [B, 4, 4] transformation matrix, the other arguments should be default!"
        xyz = to_nc(einsum("bij,bnj->bni", mat, to_gc(xyz, ndim=3)), ndim=3)
    else:
        assert trans is None or trans.ndim in [2, 3]
        assert rot is None or rot.ndim == 3
        assert scale is None or scale.ndim == 1

        for o in order:
            if o == "T" and trans is not None:
                xyz = xyz + (trans[:, None] if trans.ndim == 2 else trans)
            elif o == "R" and rot is not None:
                xyz = einsum("bij,bnj->bni", rot, xyz)
            elif o == "S" and scale is not None:
                xyz = xyz * scale[:, None, None]
    return xyz


def transform_bboxes(bboxes, trans=None, rot=None, scale=None):
    """
    bboxes: [
        center: [B, N, 3]
        extent: [B, N, 3]
        rot: [B, N, 3, 3]
        ]
    mat: [B, 4, 4]
    trans: [B, 3]
    rot: [B, 3, 3]
    scale: [B]
    """
    if bboxes is None:
        return None
    center, extent = bboxes[:2]
    rot = bboxes[2] if (len(bboxes) > 2 and bboxes[2].shape[-2] == 3 and bboxes[2].shape[-1] == 3 and bboxes[2].ndim == 4) else None
    center = transform_pcd(center, None, trans, rot, scale, "RST")
    extent = transform_pcd(extent, None, None, None, scale, "S")
    rot = transform_pcd(rot, None, rot, None, None, "R")
    if rot is not None:
        return [center, extent, rot] + bboxes[3:]
    else:
        return [center, extent] + bboxes[2:]


def apply_rot_trans(data, xyz, rot, is_vel, trans_first=False, with_rot=True, with_xyz=True):
    # xyz: [B, 3]
    # rot: [B, 3, 3]
    # data: [B, 3, N] or [B, 3] or [B, 2, N], or [B, 2]

    assert xyz.ndim == 2 and rot.ndim == 3, f"{xyz.ndim}, {rot.ndim}"
    assert data.ndim == 2 or data.ndim == 3, f"{data.ndim}"

    if data.shape[-2] == 2:
        rot = rot[..., :2, :2]
        xyz = xyz[..., :2]

    def apply_rot(x):
        if not with_rot:
            return x
        if x.ndim == 3:
            x = torch.einsum("bin,bji->bjn", x, rot)
        else:
            x = torch.einsum("bi,bji->bj", x, rot)
        return x

    def apply_trans(x):
        if not with_xyz:
            return x
        if is_vel:
            return x

        if x.ndim == 3:
            x = x + xyz[..., None]
        else:
            x = x + xyz
        return x

    if trans_first:
        data = apply_trans(data)
        data = apply_rot(data)
    else:
        data = apply_rot(data)
        data = apply_trans(data)
    return data

@AUGMENTATIONS.register_module()
class GlobalRotScaleTrans(BaseAugmentation):
    """Apply global rotation, scaling and translation to a 3D scene.

    Args:
        rot_range (list[float]): Range of rotation angle.
            Defaults to [-0.78539816, 0.78539816] (close to [-pi/4, pi/4]).
        scale_ratio_range (list[float]): Range of scale ratio.
            Defaults to [0.95, 1.05].
        translation_range (list[float]): The standard deviation of translation
            noise. This applies random translation to a scene by a noise, which
            is sampled from a gaussian distribution whose standard deviation
            is set by ``translation_range``. Defaults to [0, 0, 0]
        shift_height (bool): Whether to shift height.
            (the fourth dimension of indoor points) when scaling.
            Defaults to False.
    """

    def __init__(
        self,
        main_key=["obs/pointcloud/xyz"],
        req_keys=["obs/pointcloud/xyz", "obs/state/ee_pos", "obs/state/ee_vel", "obs/state/base_pos", "obs/state/base_vel"],
        rot_range=[-0.78539816, 0.78539816],
        rot_axis="z",
        scale_ratio_range=[0.95, 1.05],
        translation_range=[0, 0, 0],
        shift_height=False,
    ):
        super(GlobalRotScaleTrans, self).__init__(main_key, req_keys)

        if rot_range is not None:
            seq_types = (list, tuple, np.ndarray)
            if not isinstance(rot_range, seq_types):
                assert isinstance(rot_range, (int, float)), f"unsupported rot_range type {type(rot_range)}"
                rot_range = [-rot_range, rot_range]
        self.rot_range = rot_range

        assert rot_axis in ["x", "y", "z", 0, 1, 2]
        self.rot_axis = ord(rot_axis) - ord("x") if isinstance(rot_axis, str) else rot_axis

        if scale_ratio_range is not None:
            assert (
                isinstance(scale_ratio_range, seq_types) and len(scale_ratio_range) == 2
            ), f"unsupported scale_ratio_range type {type(scale_ratio_range)}"
        self.scale_ratio_range = scale_ratio_range

        if translation_range is not None:
            translation_range = torch.tensor(translation_range, dtype=torch.float)
            assert (translation_range >= 0).all(), "translation_range should be positive"
        self.translation_range = translation_range

        self.shift_height = shift_height

    def process_single(self, data, key):
        # data [B, 3] or [B, 3, N]
        if is_null(self.infos):
            batch_size = data.shape[0]
            mat = torch.zeros([batch_size, 4, 4], device=data.device)  # [B, 3, 3]
            mat[..., 3, 3] = 1

            if is_not_null(self.rot_range):
                angle = torch.zeros([batch_size, 1], device=data.device).uniform_(*self.rot_range)
                mat[..., :3, :3] = batch_rot_with_axis(angle, self.rot_axis)

            if is_not_null(self.scale_ratio_range):
                scale_factor = torch.zeros([batch_size, 3, 1], device=data.device).uniform_(*self.scale_ratio_range)
                mat[..., :3, :] *= scale_factor

            if is_not_null(self.translation_range):
                delta_xyz = (torch.rand([batch_size, 3], device=data.device) - 0.5) * 2 * self.translation_range.to(data.device)
                if not self.shift_height:
                    delta_xyz[-1] = 0
                mat[..., :3, 3] = delta_xyz
            self.infos = mat
        mat = self.infos
        if key == "obs/inst_box":
            assert not is_not_null(self.scale_ratio_range) and is_not_null(self.translation_range)
            inst_box_center, inst_box_rot = data[0], data[2]  # [B, Ninst, 3], [B, Ninst, 3, 3]
            inst_box_center = apply_rot_trans(inst_box_center, mat[..., :3, 3], mat[..., :3, :3], "vel" in key)
            inst_box_rot = torch.einsum("bij,bnjk->bnik", mat[..., :3, :3], inst_box_rot)
            data[0], data[2] = inst_box_center, inst_box_rot
            return data
        else:
            return apply_rot_trans(
                data,
                mat[..., :3, 3],
                mat[..., :3, :3],
                "vel" in key,
                with_xyz=self.translation_range is not None,
                with_rot=self.rot_range is not None,
            )

    def __repr__(self):
        """str: Return a string that describes the module."""
        repr_str = self.__class__.__name__
        if self.rot_range is not None:
            repr_str += f"(rot_range={self.rot_range},"
        if self.scale_ratio_range is not None:
            repr_str += f" scale_ratio_range={self.scale_ratio_range},"
        if self.translation_range is not None:
            repr_str += f" translation_range={self.translation_range},"
        repr_str += f" shift_height={self.shift_height})"
        return repr_str



@AUGMENTATIONS.register_module()
class RandomDownSample(BaseAugmentation):
    def __init__(self, main_key="inputs/xyz", req_keys=["input/xyz"], max_num_points=None, drop_ratio=None, fixed_ratio=True):
        super(RandomDownSample, self).__init__(main_key, req_keys)
        assert is_not_null(drop_ratio) ^ is_not_null(max_num_points)
        self.max_num_points = max_num_points
        self.drop_ratio = drop_ratio
        self.fixed_ratio = fixed_ratio

    def process_single(self, data, key):
        assert data.ndim == 3
        B, N = data.shape[0], data.shape[-1]
        import time

        if is_null(self.infos):
            if is_not_null(self.drop_ratio):
                n_drop = int(N * self.drop_ratio) if self.fixed_ratio else np.random.randint(int(N * self.drop_ratio))
                max_num_points = N - n_drop
            else:
                max_num_points = min(self.max_num_points, N)
            index = batch_perm(data[:1, 0, :], 1, max_num_points)[0]
            self.infos = (max_num_points, index)
            assert index.ndim == 1
        max_num_points, index = self.infos
        if max_num_points >= N:
            return data
        return DictArray(data).slice(index, -1, wrapper=False)
        # return DictArray(data).gather(1, index, wrapper=False)

    def __repr__(self):
        """str: Return a string that describes the module."""
        repr_str = self.__class__.__name__
        if is_not_null(self.drop_ratio):
            repr_str += f"(drop_ratio={self.drop_ratio}) (drop_ratio={self.fixed_ratio})"
        else:
            repr_str += f"(max_num_points={self.max_num_points})"
        return repr_str

@AUGMENTATIONS.register_module()
class ColorJitterPoints(BaseAugmentation):
    """Randomly jitter point colors.
    """

    def __init__(self, main_key="inputs/rgb", req_keys="inputs/rgb", brightness = 0.5, contrast = 0.5, saturation = 0.5, hue = 0.5):
        super(ColorJitterPoints, self).__init__(main_key, req_keys)
        # check the range of the brightness, contrast, saturation and hue
        if brightness < 0 or brightness > 1:
            raise ValueError("brightness shoud be non-negative")
        if contrast < 0 or contrast > 1:
            raise ValueError("contrast shoud be non-negative")
        if saturation < 0 or saturation > 1:
            raise ValueError("saturation shoud be non-negative")
        if hue < 0 or hue > 0.5:
            raise ValueError("hue shoud be non-negative")
        self.brightness = brightness
        self.contrast = contrast
        self.saturation = saturation
        self.hue = hue
        self.transform = ColorJitter(brightness, contrast, saturation, hue)

    def process_single(self, data, key):
        with torch.no_grad():
            assert data.shape[-2] == 3
            data = self.transform(data[:,:,None,:]).squeeze(-2)
            return data 
        
    def __repr__(self):
        repr_str = self.__class__.__name__
        repr_str += f"(brightness={self.brightness},"
        repr_str += f"contrast={self.contrast},"
        repr_str += f"saturation={self.saturation},"
        repr_str += f"hue={self.hue})"
        return repr_str
 

@AUGMENTATIONS.register_module()
class RandomJitterPoints(BaseAugmentation):
    """Randomly jitter point coordinates.
    Different from the global translation in ``GlobalRotScaleTrans``, here we apply different noises to each element in a scene.
    """

    def __init__(self, main_key="inputs/xyz", req_keys=None, jitter_range=[-0.1, 0.1]):
        super(RandomJitterPoints, self).__init__(main_key, req_keys)
        self.jitter_range = jitter_range 

    def process_single(self, xyz, key):
        assert xyz.shape[-2] == 3
        jitter_noise = torch.FloatTensor(*xyz.shape).uniform_(*self.jitter_range)
        # print(xyz.shape, self.jitter_std.shape)
        jitter_noise = jitter_noise.to(device=xyz.device, dtype=xyz.dtype)
        xyz = xyz + jitter_noise
        return xyz

    def __repr__(self):
        repr_str = self.__class__.__name__
        repr_str += f"(jitter_range={self.jitter_range},"
        return repr_str
    
@AUGMENTATIONS.register_module()
class AddOriginBall(BaseAugmentation):
    """Add randomly sampled points near the origin in the world for PushChair, since in ManiSkill 2021 the target
    ball indicator is always centered at the origin.
    """

    def __init__(
        self, main_key="obs/pointcloud/xyz", req_keys=["obs/pointcloud/xyz", "obs/pointcloud/rgb", "obs/pointcloud/seg"], n_pts=50, noise_std=0.02
    ):
        super(AddOriginBall, self).__init__(main_key, req_keys)
        self.n_pts = n_pts
        assert isinstance(noise_std, (int, float)), f"unsupported noise_std type {type(noise_std)}"
        self.noise_std = noise_std

    def process(self, data):
        pcd = data["obs/pointcloud"]
        B = pcd["xyz"].size(0)
        ex_xyz = torch.randn([B, self.n_pts, 3], device=pcd["xyz"].device) * self.noise_std
        ex_seg = torch.zeros([B, self.n_pts, pcd["seg"].size(-1)], device=pcd["seg"].device)
        data["obs/pointcloud/xyz"] = torch.cat([pcd["xyz"], ex_xyz], dim=1)
        data["obs/pointcloud/seg"] = torch.cat([pcd["seg"], ex_seg], dim=1)
        if "rgb" in pcd.keys():
            ex_rgb = torch.zeros([B, self.n_pts, 3], device=pcd["rgb"].device)
            data["obs/pointcloud/rgb"] = torch.cat([pcd["rgb"], ex_rgb], dim=1)
        return data

    def __repr__(self):
        repr_str = self.__class__.__name__
        repr_str += f"n_pts={self.n_pts})"
        repr_str += f"(noise_std={self.noise_std},"
        return repr_str
