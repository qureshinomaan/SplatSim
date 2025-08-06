"""
    Full model
    
    This file is deprecated and used for testing pre-trained models only.
        -- an extra constant 1 needs to be loaded after each direction queries due to some legacy reason
"""


import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader

# https://github.com/erikwijmans/Pointnet2_PyTorch
from pointnet2_ops.pointnet2_modules import PointnetFPModule, PointnetSAModule
from pointnet2.models.pointnet2_ssg_cls import PointNet2ClassificationSSG


class PointNet2SemSegSSG(PointNet2ClassificationSSG):
    def _build_model(self):
        self.SA_modules = nn.ModuleList()
        self.SA_modules.append(
            PointnetSAModule(
                npoint=1024,
                radius=0.1,
                nsample=32,
                mlp=[3, 32, 32, 64],
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
        self.FP_modules.append(PointnetFPModule(mlp=[128 + 3, 128, 128, 128]))
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

        self.mlp1 = nn.Linear(feat_dim+1, feat_dim)
        self.mlp2 = nn.Linear(feat_dim, 1)

    # feats B x F
    # output: B
    def forward(self, feats):
        one_net = torch.ones(feats.shape[:-1]).unsqueeze(-1).to(feats.device).float()
        feats = torch.cat([feats, one_net], dim=-1)
        net = F.leaky_relu(self.mlp1(feats))
        net = torch.sigmoid(self.mlp2(net)).squeeze(1)
        return net
 

class Actor(nn.Module):
    def __init__(self, feat_dim, rv_dim):
        super(Actor, self).__init__()

        self.mlp1 = nn.Linear(feat_dim+rv_dim+1, feat_dim)
        self.mlp2 = nn.Linear(feat_dim, 3+3)
    
    # pixel_feats B x F, rvs B x RV_DIM
    # output: B x 6
    def forward(self, pixel_feats, rvs):
        one_net = torch.ones(pixel_feats.shape[:-1]).unsqueeze(-1).to(pixel_feats.device).float()
        net = torch.cat([pixel_feats, rvs, one_net], dim=-1)
        net = F.leaky_relu(self.mlp1(net))
        net = self.mlp2(net).reshape(-1, 3, 2)
        net = self.bgs(net)[:, :, :2].reshape(-1, 6)
        return net
   
    # input sz bszx3x2
    def bgs(self, d6s):
        bsz = d6s.shape[0]
        b1 = F.normalize(d6s[:, :, 0], p=2, dim=1)
        a2 = d6s[:, :, 1]
        b2 = F.normalize(a2 - torch.bmm(b1.view(bsz, 1, -1), a2.view(bsz, -1, 1)).view(bsz, 1) * b1, p=2, dim=1)
        b3 = torch.cross(b1, b2, dim=1)
        return torch.stack([b1, b2, b3], dim=1).permute(0, 2, 1)

    # batch geodesic loss for rotation matrices
    def bgdR(self, Rgts, Rps):
        Rds = torch.bmm(Rgts.permute(0, 2, 1), Rps)
        Rt = torch.sum(Rds[:, torch.eye(3).bool()], 1) #batch trace
        # necessary or it might lead to nans and the likes
        theta = torch.clamp(0.5 * (Rt - 1), -1 + 1e-6, 1 - 1e-6)
        return torch.acos(theta)

    # 6D-Rot loss
    # input sz bszx6
    def get_6d_rot_loss(self, pred_6d, gt_6d):
        # [bug fixed]
        #pred_Rs = self.bgs(pred_6d.reshape(-1, 3, 2))
        #gt_Rs = self.bgs(gt_6d.reshape(-1, 3, 2))
        pred_Rs = self.bgs(pred_6d.reshape(-1, 2, 3).permute(0, 2, 1))
        gt_Rs = self.bgs(gt_6d.reshape(-1, 2, 3).permute(0, 2, 1))
        theta = self.bgdR(gt_Rs, pred_Rs)
        return theta


class Critic(nn.Module):
    def __init__(self, feat_dim):
        super(Critic, self).__init__()

        self.mlp1 = nn.Linear(feat_dim+3+3+1, feat_dim)
        self.mlp2 = nn.Linear(feat_dim, 1)

        self.BCELoss = nn.BCEWithLogitsLoss(reduction='none')

    # pixel_feats B x F, query_fats: B x 6
    # output: B
    def forward(self, pixel_feats, query_feats):
        one_net = torch.ones(query_feats.shape[:-1]).unsqueeze(-1).to(query_feats.device).float()
        net = torch.cat([pixel_feats, query_feats, one_net], dim=-1)
        net = F.leaky_relu(self.mlp1(net))
        net = self.mlp2(net).squeeze(-1)
        return net
     
    # cross entropy loss
    def get_ce_loss(self, pred_logits, gt_labels):
        loss = self.BCELoss(pred_logits, gt_labels.float())
        return loss


class Network(nn.Module):
    def __init__(self, feat_dim, rv_dim, rv_cnt):
        super(Network, self).__init__()

        self.feat_dim = feat_dim
        self.rv_dim = rv_dim
        self.rv_cnt = rv_cnt
        
        self.pointnet2 = PointNet2SemSegSSG({'feat_dim': feat_dim})
        
        self.critic = Critic(feat_dim)
        self.critic_copy = Critic(feat_dim)
        self.actor = Actor(feat_dim, rv_dim)
        self.action_score = ActionScore(feat_dim)

    # pcs: B x N x 3 (float), with the 0th point to be the query point
    def forward(self, pcs, dirs1, dirs2, gt_result):
        pcs = pcs.repeat(1, 1, 2)
        batch_size = pcs.shape[0]

        # copy the critic
        self.critic_copy.load_state_dict(self.critic.state_dict())

        # push through PointNet++
        whole_feats = self.pointnet2(pcs)

        # feats for the interacting points
        net = whole_feats[:, :, 0]  # B x F

        input_s6d = torch.cat([dirs1, dirs2], dim=1)
        input_queries = input_s6d

        # train critic
        pred_result_logits = self.critic(net, input_queries)
        critic_loss_per_data = self.critic.get_ce_loss(pred_result_logits, gt_result)

        # train actor
        rvs = torch.randn(batch_size, self.rv_cnt, self.rv_dim).float().to(net.device)
        expanded_net = net.unsqueeze(dim=1).repeat(1, self.rv_cnt, 1).reshape(batch_size*self.rv_cnt, -1)
        expanded_rvs = rvs.reshape(batch_size*self.rv_cnt, -1)
        expanded_pred_s6d = self.actor(expanded_net, expanded_rvs)
        expanded_input_s6d = input_s6d.unsqueeze(dim=1).repeat(1, self.rv_cnt, 1).reshape(batch_size*self.rv_cnt, -1)
        expanded_actor_coverage_loss_per_rv = self.actor.get_6d_rot_loss(expanded_pred_s6d, expanded_input_s6d)
        actor_coverage_loss_per_rv = expanded_actor_coverage_loss_per_rv.reshape(batch_size, self.rv_cnt)
        actor_coverage_loss_per_data = actor_coverage_loss_per_rv.min(dim=1)[0]

        with torch.no_grad():
            expanded_queries = expanded_pred_s6d
            expanded_proposal_results_logits = self.critic_copy(expanded_net.detach(), expanded_queries)
            expanded_proposal_succ_scores = torch.sigmoid(expanded_proposal_results_logits)
            proposal_succ_scores = expanded_proposal_succ_scores.reshape(batch_size, self.rv_cnt)
            avg_proposal_succ_scores = proposal_succ_scores.mean(dim=1)

        # train action_score
        pred_action_scores = self.action_score(net)
        action_score_loss_per_data = (pred_action_scores - avg_proposal_succ_scores)**2

        return critic_loss_per_data, actor_coverage_loss_per_data, action_score_loss_per_data, pred_result_logits, whole_feats
        
    # for sample_succ
    def inference_whole_pc(self, feats, dirs1, dirs2):
        num_pts = feats.shape[-1]
        batch_size = feats.shape[0]

        feats = feats.permute(0, 2, 1)  # B x N x F
        feats = feats.reshape(batch_size*num_pts, -1)

        input_queries = torch.cat([dirs1, dirs2], dim=-1)
        input_queries = input_queries.unsqueeze(dim=1).repeat(1, num_pts, 1)
        input_queries = input_queries.reshape(batch_size*num_pts, -1)

        pred_result_logits = self.critic(feats, input_queries)

        soft_pred_results = torch.sigmoid(pred_result_logits)
        soft_pred_results = soft_pred_results.reshape(batch_size, num_pts)

        return soft_pred_results
    
    def inference_action_score(self, pcs):
        pcs = pcs.repeat(1, 1, 2)
        batch_size = pcs.shape[0]
        num_point = pcs.shape[1]

        net = self.pointnet2(pcs)

        net = net.permute(0, 2, 1).reshape(batch_size*num_point, -1)
        
        pred_action_scores = self.action_score(net)
        pred_action_scores = pred_action_scores.reshape(batch_size, num_point)
        return pred_action_scores

    def inference_actor(self, pcs):
        pcs = pcs.repeat(1, 1, 2)
        batch_size = pcs.shape[0]

        whole_feats = self.pointnet2(pcs)
        net = whole_feats[:, :, 0]

        rvs = torch.randn(batch_size, self.rv_cnt, self.rv_dim).float().to(net.device)
        expanded_net = net.unsqueeze(dim=1).repeat(1, self.rv_cnt, 1).reshape(batch_size*self.rv_cnt, -1)
        expanded_rvs = rvs.reshape(batch_size*self.rv_cnt, -1)
        expanded_pred_s6d = self.actor(expanded_net, expanded_rvs)
        pred_s6d = expanded_pred_s6d.reshape(batch_size, self.rv_cnt, 6)
        return pred_s6d
    
    def inference_critic(self, pcs, dirs1, dirs2, abs_val=False):
        pcs = pcs.repeat(1, 1, 2)
        whole_feats = self.pointnet2(pcs)
        net = whole_feats[:, :, 0]

        input_queries = torch.cat([dirs1, dirs2], dim=1)
        pred_result_logits = self.critic(net, input_queries)
        if abs_val:
            pred_results = torch.sigmoid(pred_result_logits)
        else:
            pred_results = (pred_result_logits > 0)
        return pred_results
    
