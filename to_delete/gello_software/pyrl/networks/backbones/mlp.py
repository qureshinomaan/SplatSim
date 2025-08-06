# import torch.nn as nn, torch.nn.functional as F
# from torch.nn.modules.batchnorm import _BatchNorm

# from einops.layers.torch import Rearrange
# from pyrl.networks.modules import set_norm_input
from pyrl.utils.meta import ConfigDict
from pyrl.utils.torch import ExtendedModule, ExtendedSequential

from ..builder import NETWORK, build_all
from ..modules import build_init, is_ln
from ..utils import combine_obs_with_action


@NETWORK.register_module()
class MLP(ExtendedModule):
    """
    It supports linear mlp and conv mlp
    """

    def __init__(
        self,
        mlp_spec,
        block_type="Linear",
        nn_cfg=None,
        norm_cfg=dict(type="LN1d"),
        act_cfg=dict(type="ReLU"),
        bias="auto",
        inactivated_output=True,
        zero_out_indices=None,  # slice("action_shape", None, None),
        dense_init_cfg=None,  # dict(type="orthogonal_init", gain=1.414, bias=0),
        ignore_first_ln=False,
        separate_module=False,
        **kwargs,
    ):
        super(MLP, self).__init__()
        ignore_first_ln = ignore_first_ln and norm_cfg is not None and is_ln(norm_cfg)
        self.ensemble_model = nn_cfg is not None and nn_cfg.get("type", None) == "EnsembleLinear"
        self.num_modules = None
        assert block_type in ["Linear", "Conv"]
        block_cfg_default = ConfigDict(type=block_type + "Module", nn_cfg=nn_cfg, norm_cfg=norm_cfg, act_cfg=act_cfg, bias=bias, **kwargs)
        self.mlp = ExtendedSequential()

        for i in range(len(mlp_spec) - 1):
            block_cfg = block_cfg_default.copy()
            block_cfg["in_channels"] = mlp_spec[i]
            block_cfg["out_channels"] = mlp_spec[i + 1]

            if inactivated_output and i == len(mlp_spec) - 2:
                block_cfg["act_cfg"] = block_cfg["norm_cfg"] = None
            elif ignore_first_ln and i == 0:
                block_cfg["norm_cfg"] = None
            block_cfg["use_default_init"] = False
            if not separate_module:
                block_cfg["index"] = i
            block = build_all(block_cfg)
            self.mlp.extend(block, separate_module, f"layer{i}" if separate_module else None)
        if dense_init_cfg is not None or zero_out_indices is not None:
            self.reset_parameters(dense_init_cfg, zero_out_indices)

    @staticmethod
    def is_dense(module):
        module_class_name = type(module).__name__
        return "Linear" in module_class_name or "Conv" in module_class_name

    @property
    def last_dense(self):
        for i in range(len(self.mlp) - 1, -1, -1):
            if self.is_dense(self.mlp[i]):
                return self.mlp[i]
        return None

    def reset_parameters(self, dense_init_cfg, zero_out_indices):
        dense_init = build_init(dense_init_cfg)
        if dense_init is not None:
            for m in self.modules():
                if self.is_dense(m):
                    dense_init(m)
        if zero_out_indices is not None:
            last_dense = self.last_dense
            # initialized near zeros for log_std in RL actor network, https://arxiv.org/pdf/2005.05719v1.pdf fig 7.a
            if last_dense is not None:
                last_dense.weight[zero_out_indices].data.uniform_(-1e-3, 1e-3)
                last_dense.bias[zero_out_indices].data.uniform_(-1e-3, 1e-3)

    def forward(self, feature, actions=None, **kwargs):
        feature = combine_obs_with_action(feature, actions)

        if self.ensemble_model:
            import torch

            assert feature.ndim in [2, 3]
            if feature.ndim == 2 or feature.shape[1] != self.num_modules:
                feature = torch.repeat_interleave(feature[..., None, :], self.num_modules, dim=-2)
        return self.mlp(feature)


@NETWORK.register_module()
class LinearMLP(MLP):
    def __init__(self, mlp_spec, norm_cfg=None, act_cfg=dict(type="ReLU"), bias="auto", *args, **kwargs):
        super(LinearMLP, self).__init__(mlp_spec, block_type="Linear", norm_cfg=norm_cfg, act_cfg=act_cfg, bias=bias, *args, **kwargs)


@NETWORK.register_module()
class ConvMLP(MLP):
    def __init__(self, mlp_spec, norm_cfg=dict(type="LN1d"), act_cfg=dict(type="ReLU"), bias="auto", *args, **kwargs):
        super(ConvMLP, self).__init__(
            mlp_spec, block_type="Conv", norm_cfg=norm_cfg, act_cfg=act_cfg, bias=bias, nn_cfg={"type": "Conv1d"}, *args, **kwargs
        )
