"""
    ACtionability score model only
"""
from multiprocessing.connection import wait
from this import d
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader

import ipdb

# https://github.com/erikwijmans/Pointnet2_PyTorch
from pointnet2_ops.pointnet2_modules import PointnetFPModule, PointnetSAModule
from pointnet2.models.pointnet2_ssg_cls import PointNet2ClassificationSSG


class PointNet2SemSegSSG(PointNet2ClassificationSSG):
    def _build_model(self):
        self.SA_modules = nn.ModuleList()
        self.SA_modules.append(
            PointnetSAModule(
                npoint=1024,    #!1024
                radius=0.1,
                nsample=32,
                mlp=[self.hparams["input_feat"], 32, 32, 64],
                use_xyz=True,
            )
        )
        self.SA_modules.append(
            PointnetSAModule(
                npoint=256,
                radius=0.2,
                nsample=32,
                mlp=[64, 64, 64, 128],
                use_xyz=True,
            )
        )
        self.SA_modules.append(
            PointnetSAModule(
                npoint=64,
                radius=0.4,
                nsample=32,
                mlp=[128, 128, 128, 256],
                use_xyz=True,
            )
        )
        self.SA_modules.append(
            PointnetSAModule(
                npoint=16,
                radius=0.8,
                nsample=32,
                mlp=[256, 256, 256, 512],
                use_xyz=True,
            )
        )

        self.FP_modules = nn.ModuleList()
        self.FP_modules.append(PointnetFPModule(mlp=[128 + self.hparams["input_feat"], 128, 128, 128]))
        self.FP_modules.append(PointnetFPModule(mlp=[256 + 64, 256, 128]))
        self.FP_modules.append(PointnetFPModule(mlp=[256 + 128, 256, 256]))
        self.FP_modules.append(PointnetFPModule(mlp=[512 + 256, 256, 256]))

        self.fc_layer = nn.Sequential(
            nn.Conv1d(128, self.hparams['feat_dim'], kernel_size=1, bias=False),
            nn.BatchNorm1d(self.hparams['feat_dim']),
            nn.ReLU(True),
        )

    def forward(self, pointcloud):
        r"""
            Forward pass of the network

            Parameters
            ----------
            pointcloud: Variable(torch.cuda.FloatTensor)
                (B, N, 3 + input_channels) tensor
                Point cloud to run predicts on
                Each point in the point-cloud MUST
                be formated as (x, y, z, features...)
        """
        xyz, features = self._break_up_pc(pointcloud)

        xyz -= xyz.mean(axis=1, keepdim=True)
        xyz = xyz/(xyz.var(axis=1, keepdim=True)+1e-8)

        l_xyz, l_features = [xyz], [features]
        for i in range(len(self.SA_modules)):
            li_xyz, li_features = self.SA_modules[i](l_xyz[i], l_features[i])
            l_xyz.append(li_xyz)
            l_features.append(li_features)

        for i in range(-1, -(len(self.FP_modules) + 1), -1):
            l_features[i - 1] = self.FP_modules[i](
                l_xyz[i - 1], l_xyz[i], l_features[i - 1], l_features[i]
            )

        return self.fc_layer(l_features[0])


class ActionScore(nn.Module):
    def __init__(self, feat_dim):
        super(ActionScore, self).__init__()

        self.mlp1 = nn.Linear(feat_dim, feat_dim)
        self.mlp2 = nn.Linear(feat_dim, 1)

    # feats B x F
    # output: B
    def forward(self, feats):
        net = F.leaky_relu(self.mlp1(feats))
        net = torch.sigmoid(self.mlp2(net)).squeeze(1)
        return net
 

class Network(nn.Module):
    def __init__(self, input_feat, feat_dim):
        super(Network, self).__init__()

        self.input_feat = input_feat
        self.feat_dim = feat_dim
        
        self.pointnet2 = PointNet2SemSegSSG({'input_feat': input_feat, 'feat_dim': feat_dim})
        
        self.action_score = ActionScore(feat_dim)

    # pcs: B x N x 3 (float), with the 0th point to be the query point
    def forward(self, batch):  
        B = batch.shape[0]
        N = batch.shape[1]
        F = self.feat_dim
        assert(batch.shape[2] == self.input_feat)
        # num_envs * N * 4
        pcs = batch[:, :, 0:3]
        data = torch.cat([pcs, batch], dim=-1)
        # ipdb.set_trace()
        # data = torch.cat((pcs_repeat, batch[:, :, 3].reshape(batch.shape[0], batch.shape[1], 1)), dim=2)
        # ipdb.set_trace()
        # push through PointNet++
        whole_feats = self.pointnet2(data)  # B*128*N

        # train action_score
        # ipdb.set_trace()
        whole_feats_permuted = whole_feats.permute(0, 2, 1).reshape(B*N, F)         # BN * F
        pred_action_scores_permuted = self.action_score(whole_feats_permuted)       # BN * 1
        pred_action_scores = pred_action_scores_permuted.view(B, N)                 # B * N
        # output = torch.cat((pcs[:, :, 0:3], pred_action_scores.view(B, N, 1)), dim=2)

        return pred_action_scores
        # for i in range(whole_feats.shape[2]): # N
        #     # feats for the interacting points
        #     net = whole_feats[:, :, i]  # B x F
        #     pred_action_scores = self.action_score(net)
        #     output[:, i]=pred_action_scores
        #     gt_i = gt[:, i, 0]  # B*1
        #     action_score_loss_per_data = (pred_action_scores - gt_i)**2
        #     loss[i]=action_score_loss_per_data
        # output = torch.unsqueeze(output, 2)
        # # ipdb.set_trace()
        # output = torch.cat((pcs[:, :, 0:3], output), dim=2) # B*N*4
        
