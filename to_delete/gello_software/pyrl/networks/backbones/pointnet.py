"""
PointNet: Deep Learning on Point Sets for 3D Classification and Segmentation
    https://arxiv.org/abs/1612.00593
Reference Code:
    https://github.com/fxia22/pointnet.pytorch.git
"""

import numpy as np, torch, torch.nn as nn, torch.nn.functional as F, math
from copy import deepcopy

from pyrl.utils.data import dict_to_seq, split_axis, GDict, DictArray, repeat, get_dtype, to_f32
from pyrl.utils.torch import masked_average, masked_max, ExtendedModule

# from ..modules.attention import MultiHeadAttention
from ..builder import NETWORK, build_all
from .mlp import ConvMLP, LinearMLP


# Derived from https://github.com/wzlxjtu/PositionalEncoding2D/blob/master/positionalembedding2d.py
def positionEmbedding(feature_dim, num_embedding):
    assert feature_dim % 2 == 0, f"Positional encoding must be operated on even dimensional features"
    embedding = torch.zeros(num_embedding, feature_dim)
    position = torch.arange(0, num_embedding).unsqueeze(1)
    div_term = torch.exp((torch.arange(0, feature_dim, 2, dtype=torch.float) * -(math.log(10000.0) / feature_dim)))
    embedding[:, 0::2] = torch.sin(position.float() * div_term)
    embedding[:, 1::2] = torch.cos(position.float() * div_term)
    return embedding


class STNkd(ExtendedModule):
    def __init__(self, k=3, mlp_spec=[64, 128, 1024], norm_cfg=dict(type="BN1d", eps=1e-6), act_cfg=dict(type="ReLU"), ignore_first_ln=True):
        super(STNkd, self).__init__()
        self.conv = ConvMLP(
            [k] + mlp_spec, norm_cfg, act_cfg=act_cfg, inactivated_output=False, ignore_first_ln=ignore_first_ln
        )  # k -> 64 -> 128 -> 1024
        pf_dim = mlp_spec[-1]
        mlp_spec = [pf_dim // 2**i for i in range(len(mlp_spec))]
        self.mlp = LinearMLP(mlp_spec + [k * k], norm_cfg, act_cfg=act_cfg, inactivated_output=True)  # 1024 -> 512 -> 256 -> k * k
        self.k = k

    def forward(self, feature):
        assert feature.ndim == 3, f"Feature shape {feature.shape}!"
        feature = self.mlp(self.conv(feature).max(-1)[0])
        feature = split_axis(feature, 1, [self.k, self.k])
        return torch.eye(self.k, device=feature.device) + feature


class PointCloudBase(ExtendedModule):
    @torch.no_grad()
    def preprocess(self, inputs, xyz_in_feat=True):
        # assert inputs are channel-last; output is channel-first
        if isinstance(inputs, (dict, GDict)):
            xyz = inputs["xyz"].to(self.device)
            feature = [xyz] if xyz_in_feat else []
            if "rgb" in inputs:
                rgb = inputs["rgb"]
                if get_dtype(rgb) == "uint8":
                    rgb = rgb.to(self.device) / 255.0
                feature.append(rgb)
            for key in ["pos_encoding", "seg"]:
                if key in inputs:
                    feature.append(inputs[key].to(device=self.device, dtype=self.dtype))
            feature = torch.cat(feature, dim=-2) if len(feature) > 0 else None
        elif isinstance(inputs, (list, tuple)):
            xyz, feature = [], []
            for input_i in inputs:
                xyz_i, feature_i = self.preprocess(input_i)
                xyz.append(xyz_i)
                feature.append(feature_i)
        else:
            xyz = inputs.to(device=self.device, dtype=self.dtype)
            feature = xyz if xyz_in_feat else None
        return xyz, feature


@NETWORK.register_module()
class PointNet(PointCloudBase):
    def __init__(
        self,
        feat_dim,
        mlp_spec=[64, 128, 1024],
        out_channels=None,
        global_feat=True,
        feature_transform=[1],
        norm_cfg=dict(type="LN1d", eps=1e-6),
        act_cfg=dict(type="ReLU"),
        ignore_first_ln=False,  # Huge performance drop on ModelNet
        num_patch=1,
        **kwargs,
    ):
        super(PointNet, self).__init__()
        self.global_feat = global_feat
        self.feature_transform = feature_transform
        # self.num_patch = num_patch

        mlp_spec = deepcopy(mlp_spec)
        # Feature transformation in PointNet. For RL we usually do not use them.
        if 1 in feature_transform:
            self.stn = STNkd(3, mlp_spec, norm_cfg=norm_cfg, act_cfg=act_cfg, ignore_first_ln=ignore_first_ln, **kwargs)
        if 2 in feature_transform:
            self.conv1 = ConvMLP(
                [feat_dim, mlp_spec[0]], norm_cfg=norm_cfg, act_cfg=act_cfg, inactivated_output=False, ignore_first_ln=ignore_first_ln, **kwargs
            )
            self.fstn = STNkd(mlp_spec[0], mlp_spec, norm_cfg=norm_cfg, act_cfg=act_cfg, ignore_first_ln=ignore_first_ln, **kwargs)
            self.conv2 = ConvMLP(mlp_spec, norm_cfg=norm_cfg, act_cfg=act_cfg, inactivated_output=False, **kwargs)
        else:
            self.conv = ConvMLP(
                [feat_dim] + mlp_spec, norm_cfg=norm_cfg, act_cfg=act_cfg, inactivated_output=False, ignore_first_ln=ignore_first_ln, **kwargs
            )
        self.final_mlp = nn.Sequential(nn.Linear(mlp_spec[-1], out_channels), nn.LayerNorm(out_channels)) if out_channels is not None else None

    def forward(self, inputs, object_feature=True, concat_state=None, **kwargs):
        xyz, feature = self.preprocess(inputs)
        # print(feature.mean())
        # from IPython import embed; embed()

        assert not ("hand_pose" in kwargs.keys() and "obj_pose_info" in kwargs.keys())
        if "hand_pose" in kwargs.keys():
            from pytorch3d.transforms import quaternion_to_matrix

            hand_pose = kwargs.pop("hand_pose")
            # use the first hand to transform point cloud coordinates
            hand_xyz = hand_pose[:, 0, :3]
            hand_quat = hand_pose[:, 0, 3:]
            hand_rot = quaternion_to_matrix(hand_quat)
            xyz = xyz - hand_xyz[:, None, :]
            xyz = torch.einsum("bni,bij->bnj", xyz, hand_rot)
        if "obj_pose_info" in kwargs.keys():
            obj_pose_info = kwargs.pop("obj_pose_info")
            center = obj_pose_info["center"]  # [B, 3]
            xyz = xyz - center[:, None, :]
            if "rot" in obj_pose_info.keys():
                rot = obj_pose_info["rot"]  # [B, 3, 3]
                xyz = torch.einsum("bni,bij->bnj", xyz, rot)

        if 1 in self.feature_transform:
            trans = self.stn(xyz.transpose(2, 1).contiguous())
            xyz = torch.bmm(xyz, trans)
            feature = torch.cat([xyz, feature[..., 3:, :]], axis=-2)

        input_feature = feature
        if 2 in self.feature_transform:
            feature = self.conv1(feature)
            trans = self.fstn(feature)
            feature = torch.bmm(feature.transpose(1, 2).contiguous(), trans).transpose(1, 2).contiguous()
            feature = self.conv2(feature)
        else:
            feature = self.conv(feature)

        if self.global_feat:
            feature = feature.max(-1)[0]
            if self.final_mlp is not None:
                feature = self.final_mlp(feature)
        else:
            raise NotImplementedError
        
        return feature
