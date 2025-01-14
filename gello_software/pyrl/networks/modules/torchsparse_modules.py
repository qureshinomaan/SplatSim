import torch, torch.nn as nn
import torchsparse.nn.functional as spf
import torchsparse.nn as spnn
from torchsparse import PointTensor, SparseTensor
from torchsparse.nn.utils import get_kernel_offsets
from ..builder import NETWORK


@NETWORK.register_module()
class SparseDroupout(nn.Module):
    def __init__(self, p=0.5, inplace=False):
        super().__init__()
        self.droupout = nn.Dropout(p, inplace)

    def forward(self, x: SparseTensor):
        x.F = self.droupout(x.F)
        return x


spnn.Droupout = SparseDroupout


def register_sparse_modules():

    pass


def build_sparse_norm(channels, use_ln=True):
    return spnn.LayerNorm(channels, eps=1e-6) if use_ln else spnn.BatchNorm(channels)


class BasicConvolutionBlock(nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size=3, stride=1, dilation=1, use_ln=False):
        super().__init__()
        self.net = nn.Sequential(
            spnn.Conv3d(in_channels, out_channels, kernel_size=kernel_size, stride=stride, dilation=dilation, transposed=False),
            build_sparse_norm(out_channels, use_ln),
            spnn.ReLU(True),
        )

    def forward(self, x):
        return self.net(x)


class BasicDeconvolutionBlock(nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size=3, stride=1, use_ln=False):
        super().__init__()
        self.net = nn.Sequential(
            spnn.Conv3d(in_channels, out_channels, kernel_size=kernel_size, stride=stride, transposed=True),
            build_sparse_norm(out_channels, use_ln),
            spnn.ReLU(True),
        )

    def forward(self, x):
        return self.net(x)


class ResidualBlock(nn.Module):
    expansion = 1

    def __init__(self, in_channels, out_channels, kernel_size=3, stride=1, dilation=1, use_ln=False):
        super().__init__()
        self.net = nn.Sequential(
            spnn.Conv3d(in_channels, out_channels, kernel_size=kernel_size, dilation=dilation, stride=stride),
            build_sparse_norm(out_channels, use_ln),
            spnn.ReLU(True),
            spnn.Conv3d(out_channels, out_channels, kernel_size=kernel_size, dilation=dilation, stride=1),
            build_sparse_norm(out_channels, use_ln),
        )

        if in_channels == out_channels * self.expansion and stride == 1:
            self.downsample = nn.Sequential()
        else:
            if stride == 1:
                self.downsample = nn.Sequential(
                    spnn.Conv3d(in_channels, out_channels, kernel_size=1, dilation=1, stride=stride),
                    build_sparse_norm(out_channels, use_ln),
                )
            else:
                self.downsample = nn.Sequential(
                    spnn.Conv3d(in_channels, out_channels, kernel_size=kernel_size, dilation=1, stride=stride),
                    build_sparse_norm(out_channels, use_ln),
                )

        self.relu = spnn.ReLU(True)

    def forward(self, x):
        return self.relu(self.net(x) + self.downsample(x))


class Bottleneck(nn.Module):
    expansion = 4

    def __init__(self, in_channels, out_channels, kernel_size=3, stride=1, dilation=1, use_ln=False):
        super(Bottleneck, self).__init__()

        self.net = nn.Sequential(
            spnn.Conv3d(in_channels, out_channels, kernel_size=1),
            build_sparse_norm(out_channels, use_ln),
            spnn.ReLU(True),
            spnn.Conv3d(out_channels, out_channels, kernel_size=kernel_size, dilation=dilation, stride=stride),
            build_sparse_norm(out_channels, use_ln),
            spnn.ReLU(True),
            spnn.Conv3d(out_channels, out_channels * self.expansion, kernel_size=1),
            build_sparse_norm(out_channels * self.expansion, use_ln),
        )

        if in_channels == out_channels * self.expansion and stride == 1:
            self.downsample = nn.Sequential()
        else:
            if stride == 1:
                self.downsample = nn.Sequential(
                    spnn.Conv3d(in_channels, out_channels * self.expansion, kernel_size=1, dilation=1, stride=stride),
                    build_sparse_norm(out_channels * self.expansion, use_ln),
                )
            else:
                self.downsample = nn.Sequential(
                    spnn.Conv3d(in_channels, out_channels * self.expansion, kernel_size=kernel_size, dilation=1, stride=stride),
                    build_sparse_norm(out_channels * self.expansion, use_ln),
                )

        self.relu = spnn.ReLU(True)

    def forward(self, x):
        return self.relu(self.net(x) + self.downsample(x))


def build_points(xyz, feature):
    """
    xyz: [B, N, 3] or a list of [N, 3]
    feature: [B, N, C] or a list of [N, 3]
    """
    assert len(xyz) == len(feature)
    if not isinstance(feature, (list, tuple)):
        feature = [feature[i] for i in range(len(feature))]
    if not isinstance(xyz, (list, tuple)):
        xyz = [xyz[i] for i in range(len(xyz))]
    idx = torch.cat([torch.ones(xyz[i].shape[0], 1, dtype=xyz[i].dtype, device=xyz[i].device) * i for i in range(len(xyz))], dim=0)
    xyz = torch.cat(xyz, dim=0)
    xyzb = torch.cat([xyz, idx], dim=-1).contiguous()
    feature = torch.cat(feature, dim=0).contiguous()
    return PointTensor(feature, xyzb)


# z: PointTensor
# return: SparseTensor
def initial_voxelize(z, init_res, after_res):
    new_float_coord = torch.cat([(z.C[:, :3] * init_res) / after_res, z.C[:, -1].view(-1, 1)], 1)

    pc_hash = spf.sphash(torch.floor(new_float_coord).int())
    sparse_hash = torch.unique(pc_hash)
    idx_query = spf.sphashquery(pc_hash, sparse_hash)
    counts = spf.spcount(idx_query.int(), len(sparse_hash))
    inserted_coords = spf.spvoxelize(torch.floor(new_float_coord), idx_query, counts)
    inserted_coords = torch.round(inserted_coords).int()
    inserted_feat = spf.spvoxelize(z.F, idx_query, counts)

    new_tensor = SparseTensor(inserted_feat, inserted_coords, 1)
    new_tensor.cmaps.setdefault(new_tensor.stride, new_tensor.coords)
    z.additional_features["idx_query"][1] = idx_query
    z.additional_features["counts"][1] = counts
    z.C = new_float_coord
    return new_tensor


# x: SparseTensor, z: PointTensor
# return: SparseTensor
def point_to_voxel(x, z):
    if z.additional_features is None or z.additional_features.get("idx_query") is None or z.additional_features["idx_query"].get(x.s) is None:

        pc_hash = spf.sphash(torch.cat([torch.floor(z.C[:, :3] / x.s[0]).int() * x.s[0], (z.C[:, -1]).int().view(-1, 1)], 1))
        sparse_hash = spf.sphash(x.C)
        idx_query = spf.sphashquery(pc_hash, sparse_hash)
        counts = spf.spcount(idx_query.int(), x.C.shape[0])
        z.additional_features["idx_query"][x.s] = idx_query
        z.additional_features["counts"][x.s] = counts
    else:
        idx_query = z.additional_features["idx_query"][x.s]
        counts = z.additional_features["counts"][x.s]

    inserted_feat = spf.spvoxelize(z.F, idx_query, counts)
    new_tensor = SparseTensor(inserted_feat, x.C, x.s)
    new_tensor.cmaps = x.cmaps
    new_tensor.kmaps = x.kmaps

    return new_tensor


# x: SparseTensor, z: PointTensor
# return: PointTensor
def voxel_to_point(x, z, nearest=False):
    if z.idx_query is None or z.weights is None or z.idx_query.get(x.s) is None or z.weights.get(x.s) is None:
        off = get_kernel_offsets(2, x.s, 1, device=z.F.device)
        old_hash = spf.sphash(torch.cat([torch.floor(z.C[:, :3] / x.s[0]).int() * x.s[0], (z.C[:, -1:]).int()], dim=1), off)
        pc_hash = spf.sphash(x.C.to(z.F.device))
        idx_query = spf.sphashquery(old_hash, pc_hash)
        weights = spf.calc_ti_weights(z.C, idx_query, scale=x.s[0]).transpose(0, 1).contiguous()
        idx_query = idx_query.transpose(0, 1).contiguous()
        if nearest:
            weights[:, 1:] = 0.0
            idx_query[:, 1:] = -1
        new_feat = spf.spdevoxelize(x.F, idx_query, weights)
        new_tensor = PointTensor(new_feat, z.C, idx_query=z.idx_query, weights=z.weights)
        new_tensor.additional_features = z.additional_features
        new_tensor.idx_query[x.s] = idx_query
        new_tensor.weights[x.s] = weights
        z.idx_query[x.s] = idx_query
        z.weights[x.s] = weights
    else:
        new_feat = spf.spdevoxelize(x.F, z.idx_query.get(x.s), z.weights.get(x.s))
        new_tensor = PointTensor(new_feat, z.C, idx_query=z.idx_query, weights=z.weights)
        new_tensor.additional_features = z.additional_features

    return new_tensor
