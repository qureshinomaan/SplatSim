"""
End-to-End Training of Deep Visuomotor Policies
    https://arxiv.org/pdf/1504.00702.pdf
Visuomotor as the base class of all visual polices.
"""
import torch, torch.nn as nn, torch.nn.functional as F
from copy import copy, deepcopy
from pyrl.utils.meta import get_logger
from pyrl.utils.torch import ExtendedModule, ExtendedModuleList, freeze_params, unfreeze_params
from pyrl.utils.data import GDict, DictArray, recover_with_mask, is_seq_of
from .mlp import LinearMLP
from ..builder import build_all, NETWORK


@NETWORK.register_module()
class Visuomotor(ExtendedModule):
    def __init__(
        self,
        visual_nn_cfg,
        mlp_cfg,
        rnn_cfg=None,
        obs_feat_cfg=None,
        ac_feat_cfg=None,
        prev_ac_feat_cfg=None,
        freeze_visual_nn=False,
        freeze_mlp=False,
        **kwargs,
    ):
        super(Visuomotor, self).__init__()
        kwargs = dict(kwargs)

        # Feature extractor [Can be shared with other network]
        self.visual_nn = kwargs.get("visual_nn", build_all(visual_nn_cfg))
        self.obs_feat = kwargs.get("obs_feat", build_all(obs_feat_cfg))  # project state vector into a feature vector
        self.ac_feat = kwargs.get("ac_feat", build_all(ac_feat_cfg))
        # self.ac_feat = kwargs.get("ac_feat", build_all(ac_feat_cfg))

        self.rnn = kwargs.get("rnn", build_all(rnn_cfg))
        self.final_mlp = build_all(mlp_cfg)
        self.is_recurrent = self.rnn is not None

        if freeze_visual_nn:
            get_logger().warning("We freeze the visual backbone!")
            freeze_params(self.visual_nn)

        if freeze_mlp:
            get_logger().warning("We freeze the whole mlp part!")
            from .mlp import LinearMLP

            assert isinstance(self.final_mlp, LinearMLP), "The final mlp should have type LinearMLP."
            freeze_params(self.final_mlp)

        self.saved_feature = None
        self.saved_visual_feature = None

    def forward(
        self,
        obs,
        actions=None,
        feature=None,
        visual_feature=None,
        prev_actions=None,
        save_feature=False,
        detach_visual=False,
        rnn_mode="base",
        rnn_states=None,
        episode_dones=None,
        is_valid=None,
        with_robot_state=True,
        **kwargs,
    ):
        obs = copy(obs)
        assert isinstance(obs, dict), f"obs is not a dict! {type(obs)}"
        assert not (feature is not None and visual_feature is not None), f"You cannot provide visual_feature and feature at the same time!"

        self.saved_feature = None
        self.saved_visual_feature = None
        robot_state, next_rnn_state = None, None
        save_feature = save_feature or (feature is not None or visual_feature is not None)

        obs_keys = list(obs.keys())
        for key in obs_keys:
            if "_box" in key or "_seg" in key or "_sem_label" in key or key == "visual_state":
                obs.pop(key)
        for key in ["state", "agent"]:
            if key in obs:
                assert robot_state is None, f"Please provide only one robot state! Obs Keys: {obs_keys}"
                robot_state = obs.pop(key)
        if not ("xyz" in obs or "rgb" in obs or "rgbd" in obs):
            assert len(obs) == 1, f"Observations need to contain only one visual element! Obs Keys: {obs.keys()}!"
            obs = obs[list(obs.keys())[0]]

        if feature is None:
            if visual_feature is None:
                if not self.is_recurrent:
                    feat = self.visual_nn(obs)
                else:
                    obs_dict = DictArray(obs)
                    # Convert sequential observations to normal ones.
                    if is_valid is None:
                        if "xyz" in obs_dict:
                            # pointcloud
                            valid_shape = obs_dict["xyz"].shape[:-2]
                        else:
                            raise NotImplementedError

                        is_valid = torch.ones(*valid_shape, dtype=torch.bool, device=self.device)
                    else:
                        is_valid = is_valid > 0.5

                    compact_obs = obs_dict.select_with_mask(is_valid, wrapper=False)
                    feat = self.visual_nn(compact_obs)
                    feat = recover_with_mask(feat, is_valid)
                    if robot_state is not None:
                        feat = torch.cat([feat, robot_state], dim=-1)
                if detach_visual:
                    feat = feat.detach()
            else:
                feat = visual_feature

            if self.rnn is not None:
                feat = self.rnn(feat, rnn_states=rnn_states, episode_dones=episode_dones, rnn_mode=rnn_mode, prev_actions=prev_actions)
                prev_actions = None
                if rnn_mode != "base":
                    feat, next_rnn_state = feat

            if save_feature:
                self.saved_visual_feature = feat.clone()

            if robot_state is not None and with_robot_state:
                assert feat.ndim == robot_state.ndim, "Visual feature and state vector should have the same dimension!"
                feat = torch.cat([feat, robot_state], dim=-1)

            if save_feature:
                self.saved_feature = feat.clone()
        else:
            feat = feature

        if actions is not None:
            actions = self.ac_feat(actions) if self.ac_feat is not None else actions
            feat = torch.cat([feat, actions], dim=-1)

        if self.final_mlp is not None:
            feat = self.final_mlp(feat)

        return (feat, next_rnn_state) if (rnn_mode != "base" and self.is_recurrent) else feat

