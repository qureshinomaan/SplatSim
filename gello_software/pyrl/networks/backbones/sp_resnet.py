"""Sparse Convolution.

References:
    https://github.com/mit-han-lab/torchsparse/blob/master/examples/example.py
    https://github.com/mit-han-lab/e3d/blob/master/spvnas/core/models/semantic_kitti/spvcnn.py
"""

import torch.nn as nn

from torchsparse import SparseTensor
import torchsparse.nn as spnn

from .mlp import ConvMLP
from .pointnet import PointCloudBase
from ..modules.torchsparse_modules import initial_voxelize, build_points
from ..builder import NETWORK

@NETWORK.register_module()
class SparseCNN(PointCloudBase):
    def __init__(self, in_channels, voxel_size=0.1, out_channels=None, mlp_spec=[128, 256, 512]):
        super(SparseCNN, self).__init__()
        self.voxel_size = voxel_size
        self.pn_mlp = ConvMLP(
            [in_channels, 32, 32],
            norm_cfg=dict(type="LN1d"),
            act_cfg=dict(type="ReLU"),
            inactivated_output=False,
            ignore_first_ln=True,
        )

        modules = [
            spnn.Conv3d(32, mlp_spec[0], kernel_size=4, stride=2),
            spnn.LayerNorm(mlp_spec[0], eps=1e-6),
            spnn.ReLU(True),
            spnn.Conv3d(mlp_spec[0], mlp_spec[1], kernel_size=4, stride=2),
            spnn.LayerNorm(mlp_spec[1], eps=1e-6),
            spnn.ReLU(True),
            spnn.Conv3d(mlp_spec[1], mlp_spec[2], kernel_size=4, stride=2),
            spnn.LayerNorm(mlp_spec[2], eps=1e-6),
            spnn.ReLU(True),
            spnn.GlobalMaxPool(),
        ]
        if out_channels is not None:
            modules += [nn.Linear(mlp_spec[-1], out_channels), nn.LayerNorm(out_channels)]
        self.net = nn.Sequential(*modules)

    def forward(self, inputs):
        xyz, feature = self.preprocess(inputs)
        feature = self.pn_mlp(feature).transpose(1, 2).contiguous()
        xyz = xyz.transpose(1, 2).contiguous()
        z = build_points(xyz, feature)
        x: SparseTensor = initial_voxelize(z, 1.0, self.voxel_size)
        x = self.net(x)
        return x
