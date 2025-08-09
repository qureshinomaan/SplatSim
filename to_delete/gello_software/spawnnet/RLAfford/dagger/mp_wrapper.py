from queue import Empty

from RLAfford.dagger.prepare_env import prepare_isaacgym_env_expert

def env_loop(idx, cuda_gpu_idx, vulkan_gpu_idx, isaac_arg, num_gpu, listener, publisher):
    """
    Subprocess loop for a non-main environment thread.
    """
    env, _ = prepare_isaacgym_env_expert(
        isaacgym_arg_str=isaac_arg,
        num_gpu=num_gpu, idx=idx,
        cuda_gpu_idx=cuda_gpu_idx, vulkan_gpu_idx=vulkan_gpu_idx)
        
    publisher.put({'idx': idx, 'num': env.num_environments})

    while True:
        try:
            msg = listener.get(block=False)

            if msg['func'] == 'close':
                env.close()
                break

            ret_dict = {'idx': idx}
            rets = method_call(env, msg)
            ret_dict.update(rets)
            publisher.put(ret_dict)
        except Empty:
            pass  # wait

def method_call(env, msg):
    """
    Message processor to handle method calls from main environment.
    """
    ret_dict = dict()
    if msg['func'] == 'eval':
        env.task.eval()
    elif msg['func'] == 'reset':
        env.reset()
    elif msg['func'] == 'step':
        obs, _, _, info = env.step(actions=msg['args']['actions'])
        ret_dict = {'obs': obs, 'info': info}
    else:
        raise NotImplementedError

    return ret_dict

class MultiprocEnvWrapper:
    """
    Wrapper for STRICTLY multiple-GPU environment runs.
    """
    def __init__(self, num_gpus, cuda_gpu_idxes, vulkan_gpu_idxes, isaacgym_str):
        import torch.multiprocessing as mp

        assert num_gpus > 1, 'Should not use Multiprocessing Environment Wrapper for single runs.'

        self.queues = [None] + [mp.Queue() for _ in range(1, num_gpus)]
        self.main_queue = mp.Queue()
        self.num_gpus = num_gpus

        self.env_threads = [None] + [mp.Process(
        target=env_loop, args=(idx, cuda_gpu_idxes[idx], vulkan_gpu_idxes[idx],
                                isaacgym_str, num_gpus, self.queues[idx], self.main_queue,
                                )) for idx in range(1, num_gpus)]

        for i in range(1, num_gpus):
            self.env_threads[i].start()
            
        self.main_env, self.expert = prepare_isaacgym_env_expert(
            isaacgym_arg_str=isaacgym_str,
            num_gpu=num_gpus, idx=0, cuda_gpu_idx=cuda_gpu_idxes[0], vulkan_gpu_idx=vulkan_gpu_idxes[0])

        self.env_per = [0 for _ in range(num_gpus)]
        self.env_per[0] = self.main_env.num_environments
        self.num_actions = self.main_env.num_actions

        self.cuda_gpu_idxes = cuda_gpu_idxes
        self.vulkan_gpu_idxes = vulkan_gpu_idxes

        for _ in range(num_gpus - 1):
            ret = self.main_queue.get()
            self.env_per[ret['idx']] = ret['num']

        self.num_environments = sum(self.env_per)

        print("Total environments available across GPUs:", self.num_environments)

    def __getattr__(self, attr):
        """
        Mock the used methods of Gym Environments, but with parallelism.
        Else, error out.
        """
        if attr == 'task':
            return self
        elif attr in ['eval', 'reset', 'step']:
            return lambda **kwargs: self.multiproc_task(attr, kwargs)
        elif attr == 'close':
            def thread_close():
                for i in range(1, self.num_gpus):
                    self.queues[i].put({"func": "close"})

                self.main_env.close()

                for i in range(1, self.num_gpus):
                    self.env_threads[i].join()
                
            return thread_close
        else:
            raise NotImplementedError

    def multiproc_task(self, attr, kwargs):
        """
        Split a task among multiple environments. Concatenate and return results if needed (for env.step mainly).
        """
        import torch

        if attr == 'step':  # if something is missed, error will occur downstream so no need to check
            # split actions
            relevant_keys = ['successes', 'rgb', 'depth', 'proprio']
            
            all_actions = kwargs['actions']
            actions_per = torch.split(all_actions, self.env_per)

            actions_per = [actions_per[idx].to(device=f'cuda:{self.cuda_gpu_idxes[idx]}') for idx in range(self.num_gpus)]

            for i in range(1, self.num_gpus):
                msg = {'func': attr, 'args': {'actions': actions_per[i]}}
                self.queues[i].put(msg)

            msg = {'func': attr, 'args': {'actions': actions_per[0]}}
            main_ret = method_call(self.main_env, msg)

            empty_list = [None for _ in range(self.num_gpus)]

            final_ret = {'obs': empty_list.copy(), 'info': {k: empty_list.copy() for k in relevant_keys}}

            final_ret['obs'][0] = main_ret['obs']

            for k in relevant_keys:
                final_ret['info'][k][0] = main_ret['info'][k]

            for _ in range(self.num_gpus - 1):
                ret_dict = self.main_queue.get()
                idx = ret_dict['idx']

                final_ret['obs'][idx] = ret_dict['obs'].to(device=f'cuda:{self.cuda_gpu_idxes[0]}')

                for k in relevant_keys:
                    final_ret['info'][k][idx] = ret_dict['info'][k].to(device=f'cuda:{self.cuda_gpu_idxes[0]}')

            final_ret['obs'] = torch.cat(final_ret['obs'])

            for k in relevant_keys:
                final_ret['info'][k] = torch.cat(final_ret['info'][k])

            return final_ret['obs'], None, None, final_ret['info']

        else:  # nothing to return, risk of missing something so do a check for sanity at the end
            dones = [True] + [False for _ in range(1, self.num_gpus)]

            for i in range(1, self.num_gpus):
                self.queues[i].put({'func': attr})

            method_call(self.main_env, {'func': attr})

            for _ in range(self.num_gpus - 1):
                dones[self.main_queue.get()['idx']] = True
            
            assert all(dones), 'Duplicate dones in processing'