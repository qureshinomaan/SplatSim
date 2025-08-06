import os
import torch
import w2a_utils

from pointnet2_ops.pointnet2_utils import furthest_point_sample

class where2act_net:
    def __init__(self, device, chkpt=None):

        self.exp_name=chkpt #"finalexp-model_all_final-pulling-None-train_all_v1" 
        self.model_epoch=81 
        self.model_version="model_3d_legacy"
        self.device = device
        self.chkpt_root = 'Collision_Predictor_Module/where2act/code/logs'
        # load train config
        self.train_conf = torch.load(os.path.join(self.chkpt_root, self.exp_name, 'conf.pth'), map_location=self.device)

        # load model
        self.model_def = w2a_utils.get_model_module(self.model_version)

        # set up device
        self.device = torch.device(self.device)

        # create models
        self.network = self.model_def.Network(self.train_conf.feat_dim, self.train_conf.rv_dim, self.train_conf.rv_cnt)

        # load pretrained model
        data_to_restore = torch.load(os.path.join(self.chkpt_root, self.exp_name, 'ckpts', '%d-network.pth' % self.model_epoch), map_location=self.device)
        self.network.load_state_dict(data_to_restore, strict=False)
        self.network.to(self.device)
        self.network.eval()



    def predict_affordance(self,pc):
        # pc_= pc-pc.mean(dim=1, keep_dims=True)
        # send to device
        # push through une
        # print(pc.shape)
        with torch.no_grad():
            # push through the network
            pred_action_score_map = self.network.inference_action_score(pc)
        return pred_action_score_map
