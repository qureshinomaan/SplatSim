"""
IMPALA:
    Paper: Scalable Distributed Deep-RL with Importance Weighted Actor
    Reference code: https://github.com/facebookresearch/torchbeast

Nauture CNN:
    Code: https://github.com/hill-a/stable-baselines/blob/master/stable_baselines/common/policies.py
"""


import torch, torch.nn as nn, torch.nn.functional as F
from torch.nn import Conv2d

import numpy as np
from collections import OrderedDict

from pyrl.utils.data import to_f32, GDict, get_dtype
from pyrl.utils.torch import ExtendedModule, ExtendedSequential
from pyrl.utils.meta import ConfigDict

from ..builder import NETWORK, build_all
from ..modules import build_init, need_bias, is_ln, conv_out_shape, padding_shape


class CNNBase(ExtendedModule):
    @classmethod
    @torch.no_grad()
    def preprocess(cls, inputs):
        # assert inputs are channel-last; output is channel-first
        if isinstance(inputs, dict):
            feature = []
            if "rgb" in inputs:
                rgb = inputs["rgb"]
                if get_dtype(rgb) == "uint8":
                    rgb = to_f32(rgb) / 255.0
                feature.append(rgb)
            if "depth" in inputs:
                feature.append(to_f32(inputs["depth"]))
            if "xyz" in inputs:
                feature.append(to_f32(inputs["xyz"]))
            if "seg" in inputs:
                feature.append(to_f32(inputs["seg"]))
            feature = torch.cat(feature, dim=-3)
        elif get_dtype(inputs) == "uint8":
            feature = to_f32(inputs) / 255.0
        else:
            feature = inputs
        return feature


@NETWORK.register_module()
class IMPALA(CNNBase):
    def __init__(self, in_channel, num_pixels, out_feature_size=256, out_channel=None):
        super(IMPALA, self).__init__()
        self.in_channel = in_channel
        self.out_channel = out_channel

        self.feat_convs = []
        self.resnet1 = []
        self.resnet2 = []
        self.convs = []
        fcs = [64, 64, 64]

        self.stem = nn.Conv2d(in_channel, fcs[0], kernel_size=4, stride=4)
        in_channel = fcs[0]
        for num_ch in fcs:
            feats_convs = []
            feats_convs.append(
                nn.Conv2d(
                    in_channels=in_channel,
                    out_channels=num_ch,
                    kernel_size=3,
                    stride=1,
                    padding=1,
                )
            )
            feats_convs.append(nn.MaxPool2d(kernel_size=3, stride=2, padding=1))
            self.feat_convs.append(nn.Sequential(*feats_convs))
            in_channel = num_ch
            for i in range(2):
                resnet_block = []
                resnet_block.append(nn.ReLU())
                resnet_block.append(
                    nn.Conv2d(
                        in_channels=in_channel,
                        out_channels=num_ch,
                        kernel_size=3,
                        stride=1,
                        padding=1,
                    )
                )
                resnet_block.append(nn.ReLU())
                resnet_block.append(
                    nn.Conv2d(
                        in_channels=in_channel,
                        out_channels=num_ch,
                        kernel_size=3,
                        stride=1,
                        padding=1,
                    )
                )
                if i == 0:
                    self.resnet1.append(nn.Sequential(*resnet_block))
                else:
                    self.resnet2.append(nn.Sequential(*resnet_block))

        self.feat_convs = nn.ModuleList(self.feat_convs)
        self.resnet1 = nn.ModuleList(self.resnet1)
        self.resnet2 = nn.ModuleList(self.resnet2)
        self.img_feat_size = num_pixels // (4**3 * 16) * fcs[-1]

        self.fc = nn.Linear(self.img_feat_size, out_feature_size)
        self.final = nn.Linear(out_feature_size, self.out_channel) if out_channel else None

    def forward(self, inputs, **kwargs):
        feature = self.preprocess(inputs)
        x = self.stem(feature)
        res_input = None
        for i, fconv in enumerate(self.feat_convs):
            x = fconv(x)
            res_input = x
            x = self.resnet1[i](x)
            x += res_input
            res_input = x
            x = self.resnet2[i](x)
            x += res_input
        x = F.relu(x)
        x = x.reshape(x.shape[0], self.img_feat_size)
        x = F.relu(self.fc(x))
        if self.final:
            x = self.final(x)
        return x

@NETWORK.register_module()
class NatureCNN(ExtendedSequential, CNNBase):
    # DQN
    def __init__(
        self,
        in_channels,
        image_size,
        out_channels=512,  # Flattened feature size
        mlp_spec=[32, 64, 64],
        kernel_size=[8, 4, 2],
        stride=[4, 2, 1],
        padding=None,
        nn_cfg=None,
        norm_cfg=None,
        act_cfg=dict(type="ReLU"),
        ignore_first_ln=True,
        flatten=True,
        inactivate_output=False,
        conv_init_cfg=None,
    ):
        super(NatureCNN, self).__init__()
        ignore_first_ln = ignore_first_ln and norm_cfg is not None and is_ln(norm_cfg)

        cfg = ConfigDict(
            type="ConvModule",
            nn_cfg=nn_cfg,
            norm_cfg=norm_cfg,
            act_cfg=act_cfg,
            bias="auto",
        )
        padding = padding or [0] * len(kernel_size)
        flatten = flatten or out_channels is not None

        first_norm_cfg = None if ignore_first_ln else norm_cfg
        cfg["norm_cfg"] = first_norm_cfg
        cfg.update(dict(in_channels=in_channels, out_channels=mlp_spec[0], kernel_size=kernel_size[0], stride=stride[0], padding=padding[0], index=0))
        self.extend(build_all(cfg))

        cfg["norm_cfg"] = norm_cfg
        for i in range(1, len(mlp_spec)):
            if i == len(mlp_spec) - 1 and inactivate_output and out_channels is None:
                cfg["act_cfg"] = None
            cfg.update(
                dict(in_channels=mlp_spec[i - 1], out_channels=mlp_spec[i], kernel_size=kernel_size[i], stride=stride[i], padding=padding[i], index=i)
            )
            self.extend(build_all(cfg))
        if flatten:
            self.add_module("flatten", nn.Flatten(1))

        if out_channels is not None:
            with torch.no_grad():
                image = torch.zeros([1] + [in_channels] + list(image_size), device=self.device)
                feature_size = self(image).shape[-1]

            num = len(mlp_spec)
            self.add_module(f"linear{num}", nn.Linear(feature_size, out_channels))
            if not inactivate_output:
                self.add_module(f"act{num}", build_all(act_cfg))

        self.init_weights(conv_init_cfg)

    def init_weights(self, conv_init_cfg=None):
        if conv_init_cfg is None:
            return
        init = build_init(conv_init_cfg)
        for conv in self:
            if isinstance(conv, Conv2d):
                init(conv)

    def forward(self, images, **kwargs):
        images = self.preprocess(images)
        return ExtendedSequential.forward(self, images)


@NETWORK.register_module()
class DMCEncoder(NatureCNN):
    # For dm_control, SAC-AE
    def __init__(
        self,
        in_channels,
        image_size,
        out_channels=50,
        mlp_spec=[32, 32, 32, 32],
        kernel_size=[3, 3, 3, 3],
        stride=[2, 1, 1, 1],
        **kwargs,
    ):
        super(DMCEncoder, self).__init__(in_channels, image_size, out_channels, mlp_spec, kernel_size, stride, inactivate_output=True, **kwargs)
        self.add_module(f"norm{len(mlp_spec)}", nn.LayerNorm(out_channels))
