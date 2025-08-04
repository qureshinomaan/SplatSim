from torch.nn import Module, ModuleList, Sequential
from torch.nn.parallel import DistributedDataParallel as DDP
from torch.nn.utils import parameters_to_vector
from contextlib import contextmanager
import torch

from pyrl.utils.data import GDict, to_torch, is_seq_of
from .misc import run_with_mini_batch


class ExtendedModuleBase(Module):
    def __init__(self, *args, **kwargs):
        super(ExtendedModuleBase, self).__init__(*args, **kwargs)
        self._in_test = False  # For RL test mode ( do not update obs_rms and rew_rms )
        self.is_recurrent = False

    def set_mode(self, mode="train"):
        self._in_test = mode == "test"
        for module in self.children():
            if isinstance(module, ExtendedModuleBase):
                module.set_mode(mode)
        return self

    @property
    def dtype(self):
        return next(self.parameters()).dtype

    @property
    def trainable_parameters(self):
        return [_ for _ in self.parameters() if _.requires_grad]

    @property
    def size_trainable_parameters(self):
        return GDict([_ for _ in self.parameters() if _.requires_grad]).nbytes_all

    @property
    def num_trainable_parameters(self):
        return sum([_.numel() for _ in self.parameters() if _.requires_grad])

    @property
    @torch.no_grad()
    def grad_norm(self, ord=2):
        grads = [torch.norm(_.grad.detach(), ord) for _ in self.parameters() if _.requires_grad and _.grad is not None]
        ret = torch.norm(torch.stack(grads), ord).item() if len(grads) > 0 else 0.0
        return ret

    @torch.no_grad()
    def vector_parameters(self):
        return parameters_to_vector(self.parameters())

    def pop_attr(self, name):
        if hasattr(self, name):
            ret = getattr(self, name)
            setattr(self, name, None)
            return ret
        else:
            return None


class ExtendedModule(ExtendedModuleBase):
    # DDP has attribute device!!!!
    @property
    def device(self):
        return next(self.parameters()).device

    @contextmanager
    def no_sync(self):
        yield


class ExtendedModuleList(ModuleList, ExtendedModule):
    pass


class ExtendedSequential(Sequential, ExtendedModule):
    def extend(self, modules, separate_module=False, separate_name=None, to_beginning=False):
        assert isinstance(modules, (Sequential, Module)) or is_seq_of(modules, (Sequential, Module))
        if not separate_module:
            separate_name = None
        if isinstance(modules, (list, tuple)):
            assert separate_name is None or len(modules) == 1, "You can only specify separate_name when extending a single module."
            if to_beginning:
                modules = reversed(modules)

            for module in modules:
                self.extend(module, separate_module, separate_name, to_beginning)
        else:
            if separate_module:
                if separate_name is None:
                    assert not to_beginning, "If you want to add module to the beginning, please provide the name of the module."
                    separate_name = str(len(self))
                self.add_module(separate_name, modules)
                if to_beginning:
                    self._modules.move_to_end(separate_name, last=False)
            else:
                modules = modules._modules
                keys = list(modules.keys())
                for key in keys:
                    self.add_module(key, modules[key])
                if to_beginning:
                    for key in reversed(keys):
                        self._modules.move_to_end(key, last=False)


class ExtendedDDP(DDP, ExtendedModuleBase):
    def __getattr__(self, name):
        try:
            return super().__getattr__(name)
        except AttributeError:
            return getattr(self.module, name)
        
class BaseAgent(ExtendedModule):
    def __init__(self, *args, **kwargs):
        super(BaseAgent, self).__init__(*args, **kwargs)
        self._device_ids = None
        self._be_data_parallel = False
        self._tmp_attrs = {}
        self._use_zhiao_gae = False

        self.obs_processor = None
        self.obs_rms = None
        self.rew_rms = None
        self.batch_size = None
        self.recurrent_horizon = -1

    def reset(self, *args, **kwargs):
        pass

    @property
    def recurrent_kwargs(self):
        return dict(is_recurrent=self.is_recurrent, recurrent_horizon=self.recurrent_horizon)

    @property
    def has_obs_process(self):
        return self.obs_rms is not None or self.obs_processor is not None

    @torch.no_grad()
    def process_obs(self, data, **kwargs):
        for key in ["obs", "next_obs"]:
            if key in data:
                if self.obs_rms is not None:
                    data[key] = run_with_mini_batch(self.obs_rms.normalize, data[key], **kwargs, device=self.device, wrapper=False)
                if self.obs_processor is not None:
                    data[key] = run_with_mini_batch(self.obs_processor, {"obs": data[key]}, **kwargs)["obs"]
        return data

    @torch.no_grad()
    def forward(self, obs, **kwargs):
        obs = GDict(obs).to_torch(device=self.device, non_blocking=True, wrapper=False)
        kwargs = dict(kwargs)
        for key in ["prev_actions"]:
            if key in kwargs:
                kwargs[key] = GDict(kwargs[key]).to_torch(device=self.device, non_blocking=True, wrapper=False)

        if self.obs_rms is not None:
            obs = self.obs_rms.normalize(obs) if self._in_test else self.obs_rms.add(obs)
        if self.obs_processor is not None:
            obs = self.obs_processor({"obs": obs})["obs"]
        return self.actor(obs, **kwargs)

    def run_actor(self, obs, **kwargs):
        # @mini_batch(False)
        def run(obs, **kwargs):
            rnn_mode = kwargs.get("rnn_mode", "base")
            actor_mode = mode = kwargs.pop("mode", "explore")
            if mode == "ppo":
                assert "actions" in kwargs
                actor_mode = "dist"
                actions = kwargs.pop("actions")

            ret = self.actor(obs, **kwargs, mode=actor_mode)
            if rnn_mode != "base":
                ret, rnn_states = ret[0], ret[1]

            if mode == "ppo":
                logp = ret.log_prob(actions)
                ret = [ret, logp]
            return [ret, rnn_states] if rnn_mode != "base" else ret

        return run_with_mini_batch(
            run,
            obs=obs,
            **kwargs,
            device=self.device,
            wrapper=False,
            **self.recurrent_kwargs,
        )

    def run_critic(self, obs, actions=None, **kwargs):
        return run_with_mini_batch(self.critic, obs=obs, actions=actions, **kwargs, device=self.device, **self.recurrent_kwargs)

    @torch.no_grad()
    def compute_gae(
        self, obs, next_obs, rewards, dones, episode_dones, prev_actions=None, actions=None, ignore_dones=False, update_rms=True, batch_size=None
    ):
        """
        High-Dimensional Continuous Control Using Generalized Advantage Estimation
            https://arxiv.org/abs/1506.02438
        """
        rewards = to_torch(rewards, device=self.device, non_blocking=True)
        dones = to_torch(dones, device=self.device, non_blocking=True)
        episode_dones = to_torch(episode_dones, device=self.device, non_blocking=True)
        episode_masks = 1.0 - episode_dones.float()
        # get_logger().info(type(episode_dones))

        with self.no_sync(mode="critic"):
            values, [rnn_states, next_rnn_states, rnn_final_states] = self.run_critic(
                obs=obs,
                prev_actions=prev_actions,
                batch_size=batch_size,
                wrapper=False,
                rnn_mode="full_states",
                episode_dones=episode_dones,
            )

            next_values = self.run_critic(
                obs=next_obs,
                prev_actions=actions,
                batch_size=batch_size,
                rnn_states=next_rnn_states,
                wrapper=False,
            )
        if self.rew_rms is not None:
            std = self.rew_rms.std
            values = values * std
            next_values = next_values * std

        if not ignore_dones:
            next_values = next_values * (1.0 - dones.float())
        advantages = torch.zeros(len(rewards), 1, device=self.device, dtype=torch.float32)

        if self._use_zhiao_gae:
            sum_lambda, sum_reward, sum_end_v = 0, 0, 0
            for i in range(len(rewards) - 1, -1, -1):
                sum_lambda = sum_lambda * episode_masks[i]
                sum_reward = sum_reward * episode_masks[i]
                sum_end_v = sum_end_v * episode_masks[i]

                sum_lambda = 1.0 + self.lmbda * sum_lambda
                sum_reward = self.lmbda * self.gamma * sum_reward + sum_lambda * rewards[i]
                sum_end_v = self.lmbda * self.gamma * sum_end_v + self.gamma * next_values[i]
                sumA = sum_reward + sum_end_v
                advantages[i] = sumA / sum_lambda - values[i]
        else:
            delta = rewards + next_values * self.gamma - values
            coeff = episode_masks * self.gamma * self.lmbda
            gae = 0
            for i in range(len(rewards) - 1, -1, -1):
                gae = delta[i] + coeff[i] * gae
                advantages[i] = gae
        returns = advantages + values

        ret = {
            "old_values": values,
            "old_next_values": next_values,
            "original_returns": returns,
            "returns": returns,
            "advantages": advantages,
        }

        if self.is_recurrent:
            ret.update({"rnn_states": rnn_states, "next_rnn_states": next_rnn_states})

        if self.rew_rms is not None:
            if update_rms:
                assert self.rew_rms.training
                self.rew_rms.add(ret["returns"])
                self.rew_rms.sync()
            std = self.rew_rms.std

            ret["old_values"] = ret["old_values"] / std
            ret["old_next_values"] = ret["old_next_values"] / std
            ret["returns"] = ret["returns"] / std
            """
            print(std)
            print(
                values.mean(),
                values.std(),
                next_values.mean(),
                next_values.std(),
                returns.mean(),
                returns.std(),
            )
            print((next_values * self.gamma - values).mean())
            """
        ret = GDict(ret).to_numpy()
        torch.cuda.empty_cache()
        return ret

    def actor_grad(self, with_shared=True):
        ret = {}
        if getattr(self, "actor", None) is None:
            return ret
        ret[f"grad/actor_grad_norm"] = self.actor.grad_norm
        if with_shared:
            assert self.shared_
            if getattr(self.actor_grad.backbone, "visual_nn", None) is not None:
                ret["grad/visual_grad"] = self.actor.backbone.visual_nn.grad_norm
            from pyrl.networks import RNN

            if getattr(self.actor.backbone, "rnn", None) is not None and not isinstance(self.actor.backbone, RNN):
                ret["grad/rnn_grad"] = self.actor.backbone.rnn.grad_norm
            if self.actor.final_mlp is not None:
                ret["grad/mlp_grad"] = self.actor.final_mlp.grad_norm

    def critic_grad(self, with_shared=True):
        ret = {}
        if getattr(self, "critic", None) is None:
            return ret
        ret[f"grad/critic_grad_norm"] = self.critic.grad_norm
        if with_shared:
            assert self.shared_
            if getattr(self.actor_grad.backbone, "visual_nn", None) is not None:
                ret["grad/visual_grad"] = self.actor.backbone.visual_nn.grad_norm
            from pyrl.networks import RNN

            if getattr(self.actor.backbone, "rnn", None) is not None and not isinstance(self.actor.backbone, RNN):
                ret["grad/rnn_grad"] = self.actor.backbone.rnn.grad_norm
            if self.actor.final_mlp is not None:
                ret["grad/mlp_grad"] = self.actor.final_mlp.grad_norm

    def to_ddp(self, device_ids=None):
        self._device_ids = device_ids
        self.recover_ddp()

    def to_normal(self):
        if self._be_data_parallel and self._device_ids is not None:
            self._be_data_parallel = False
            for module_name in dir(self):
                item = getattr(self, module_name)
                if isinstance(item, DDP):
                    setattr(self, module_name, item.module)

    def recover_ddp(self):
        if self._device_ids is None:
            return
        self._be_data_parallel = True
        for module_name in dir(self):
            item = getattr(self, module_name)
            if isinstance(item, ExtendedModule) and len(item.trainable_parameters) > 0:
                if module_name not in self._tmp_attrs:
                    self._tmp_attrs[module_name] = ExtendedDDP(item, device_ids=self._device_ids, find_unused_parameters=True)
                setattr(self, module_name, self._tmp_attrs[module_name])

    def is_data_parallel(self):
        return self._be_data_parallel

    def no_sync(self, mode="actor"):
        return getattr(self, mode).no_sync()


def async_no_grad_pi(pi):
    import torch

    def run(*args, **kwargs):
        with pi.no_sync():
            with torch.no_grad():
                return pi(*args, **kwargs)

    return run


class FreezeParameters:
    """
    Modified from https://github.com/juliusfrost/dreamer-pytorch/blob/47bd509ab5cffa95ec613fd788d7ae1fe664ecd5/dreamer/utils/module.py#L17
    """

    def __init__(self, modules, *args):
        if not isinstance(modules, (list, tuple)):
            modules = [modules]
        modules = modules + list(args)
        self.modules = modules
        parameters = []
        for module in modules:
            parameters += list(module.parameters())
        self.parameters = parameters
        self.param_states = [p.requires_grad for p in parameters]

    def __enter__(self):
        for p in self.parameters:
            p.requires_grad = False

    def __exit__(self, exc_type, exc_val, exc_tb):
        for p, requires_grad in zip(self.parameters, self.param_states):
            p.requires_grad = requires_grad
