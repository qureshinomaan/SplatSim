import numpy as np
from pyrl.utils.data import DictArray, GDict

from .builder import SAMPLING


@SAMPLING.register_module()
class SamplingStrategy:
    def __init__(self, with_replacement=True, capacity=None, no_random=False, seed=None):
        self.with_replacement = with_replacement
        self.no_random = no_random
        if no_random:
            assert not with_replacement, "Fix order only supports without-replacement!"
        self.horizon = 1
        self.capacity = capacity  # same as replay buffer capacity, i.e. # of 1-steps
        self.position = 0
        self.running_count = 0
        self.seed = np.random.randint(0, 2**32 - 1) if seed is None else seed
        self.np_random = np.random.RandomState(self.seed)

        # For without replacement
        self.items = None
        self.item_index = 0
        self.need_update = False

    def get_index(self, batch_size, capacity=None, drop_last=True, auto_restart=True):
        if capacity is None:  # For 1-Step Transition, capacity is the number of data samples
            capacity = len(self)
        # For T step transition, capacity is the number of valid trajectories, and should be specified in the capacity arg
        if self.with_replacement:
            return self.np_random.randint(low=0, high=capacity, size=batch_size)

        if self.items is None or self.need_update:
            self.need_update = False
            self.items = np.arange(capacity)
            if not self.no_random:
                self.np_random.shuffle(self.items)
            self.item_index = 0
        min_query_size = batch_size if drop_last else 1
        if self.item_index + min_query_size > capacity:
            if not auto_restart:
                return None
            if not self.no_random:
                self.np_random.shuffle(self.items)
            self.item_index = 0
        else:
            batch_size = min(batch_size, capacity - self.item_index)
        index = self.items[self.item_index : self.item_index + batch_size]
        self.item_index += batch_size
        return index

    def __len__(self):
        return min(self.running_count, self.capacity)

    def restart(self):
        self.item_index = 0
        self.items = None

    def reset_all(self):
        raise NotImplementedError

    def push_batch(self, items: DictArray):
        raise NotImplementedError

    def push(self, item: GDict):
        raise NotImplementedError

    def sample(self, batch_size):
        raise NotImplementedError


@SAMPLING.register_module()
class OneStepTransition(SamplingStrategy):
    # Sample 1-step transitions. A special case of TStepTransition with better speed.
    def __init__(self, **kwargs):
        super(OneStepTransition, self).__init__(**kwargs)

    def reset(self):
        self.position = 0
        self.running_count = 0
        self.restart()

    def push_batch(self, items):
        self.need_update = True
        self.running_count += len(items)
        self.position = (self.position + len(items)) % self.capacity

    def push(self, item):
        self.need_update = True
        self.running_count += 1
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size, drop_last=True, auto_restart=True):
        # Return: index and valid masks
        index = self.get_index(batch_size, len(self), drop_last=drop_last, auto_restart=auto_restart)
        # print(index)
        # exit(0)
        if index is None:
            return None, None
        else:
            return index, np.ones([index.shape[0], self.horizon], dtype=np.bool_)


@SAMPLING.register_module()
class TStepTransition(SamplingStrategy):
    # Sample 1-step transitions. A special case of TStepTransition with better speed.

    def __init__(self, horizon=1, **kwargs):
        super(TStepTransition, self).__init__(**kwargs)
        """
        horizon means the length of the successive transitions when we do sampling.
        horizon=1: single step (for off-policy RL)
        horizon=T: T-step (for on-policy RL)
        horizon=-1: whole episode (recurrent policy)
        """

        self.horizon = horizon
        self.worker_indices = np.zeros(self.capacity, dtype=np.int16) - 1

        self.num_procs = 0
        self.current_episode = []  # The index of current episode from each worker
        self.valid_seq = []  # All valid T-step transition index. If T = -1, we will store the whole episode.
        self.dones = []  # The done of current episode from each worker

    def reset(self):
        self.position = 0
        self.running_count = 0
        self.worker_indices = self.worker_indices * 0 - 1

        self.num_procs = 0
        self.current_episode = []
        self.valid_seq = []
        self.restart()

    def push_batch(self, items):
        self.need_update = True
        episode_dones = items["episode_dones"]
        is_truncated = items["is_truncated"]
        worker_indices = items["worker_indices"]
        for i in range(episode_dones.shape[0]):
            self.push(GDict(dict(episode_dones=episode_dones[i], worker_indices=worker_indices[i], is_truncated=is_truncated[i])))

    def __len__(self):
        return np.sum([len(_) for _ in self.valid_seq])

    def push(self, item):
        self.need_update = True
        # get arr[0] from arr, an array with a single element
        worker_indices = item["worker_indices"][0]
        dones = item["episode_dones"][0] or item["is_truncated"][0]

        if worker_indices + 1 > self.num_procs:
            # Init a information pool for each worker in multiple process
            for i in range(worker_indices + 1 - self.num_procs):
                self.current_episode.append(
                    []
                )  # self.current_episode[worker_indices] = array of (positions in the replay buffer) for the current episode of the worker
                self.dones.append([])
                self.valid_seq.append(
                    []
                )  # self.valid_seq[worker_indices][i][:] = (positions in the replay buffer) for the ith valid trajectory of the worker
            self.num_procs = worker_indices + 1

        """
        if dones:
            for i in range(self.num_procs):
                print(self.running_count, i, len(self.current_episode[i]), len(self.valid_seq[i]), self.horizon,  self.capacity)
            print( item['episode_dones'][0], item['is_truncated'][0], worker_indices, self.position, self.worker_indices, self.capacity)
        # """

        # for i in range(self.num_procs):
        # print(self.running_count, dones, i, len(self.current_episode[i]), len(self.valid_seq[i]), self.horizon,  self.capacity)

        # self.current_episode and self.valid_seq[idx] are circular buffers of trajectories
        # if we are exhaust the replay buffer capacity and go back to the beginning of the replay buffer,
        # remove the old trajectories in the circular buffers that have self.position as the first index, since they have become invalid
        if self.worker_indices[self.position] >= 0:
            # Pop original item in the buffer
            last_index = self.worker_indices[self.position]

            if len(self.current_episode[last_index]) > 0 and self.position == self.current_episode[last_index][0]:
                self.current_episode[last_index].pop(0)

            if self.position == self.valid_seq[last_index][0][0]:
                # The first element in the first valid transitions
                if self.horizon > 0:
                    # Just remove for horizon > 0
                    self.valid_seq[last_index].pop(0)
                else:
                    # When horizon is -1, it stores the whole episode. I am not sure if I should delete the whole episode TODO:Check!
                    self.valid_seq[last_index][0].pop(0)
                    if len(self.valid_seq[last_index][0]) == 0:
                        self.valid_seq[last_index].pop(0)

        self.current_episode[worker_indices].append(self.position)
        self.worker_indices[self.position] = worker_indices

        if self.horizon > 0:
            if len(self.current_episode[worker_indices]) >= self.horizon:
                self.valid_seq[worker_indices].append(self.current_episode[worker_indices][-self.horizon :])
        else:
            if dones:
                self.valid_seq[worker_indices].append(self.current_episode[worker_indices])
        if dones:
            self.current_episode[worker_indices] = []

        self.running_count += 1
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size, drop_last=True, auto_restart=True):
        # Ret: [B, H], indices for each trajectory in the (batch_size) number of trajectories sampled;
        # Mask: [B, H, 1], whether a timestep in a trajectory is valid (for padding purposes)
        query_size = np.cumsum([len(_) for _ in self.valid_seq])
        length = query_size[-1]
        if length < len(self) * 0.8 and self.horizon != -1:
            print(
                f"{len(self) - length}/{len(self)} samples will be throwed out when sampling with horizon {self.horizon}, Please double check the code!"
            )
            exit(0)

        index = self.get_index(batch_size, query_size[-1], drop_last, auto_restart)
        # print(tmp.shape, tmp[0, ..., 0], index)
        # print(tmp[0, ..., 0][index])

        # exit(0)
        if index is None:
            return None, None

        ret = []
        for i in index:
            for j in range(self.num_procs):
                if i < query_size[j]:
                    break
            last_indices = 0 if j == 0 else query_size[j - 1]
            ret.append(self.valid_seq[j][i - last_indices])
        padded_size = max([len(_) for _ in ret])
        mask = np.zeros([len(ret), padded_size, 1], dtype=np.bool_)
        for i in range(len(ret)):
            mask[i, : len(ret[i])] = True
            ret[i] = ret[i] + [ret[i][0],] * (
                padded_size - len(ret[i])
            )  # adding two lists
        ret = np.array(ret, dtype=np.int)
        # print(ret)
        # exit(0)
        return ret, mask
