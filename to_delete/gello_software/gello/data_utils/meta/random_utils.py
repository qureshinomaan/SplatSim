import random, string, time, numpy as np

def random_id_generator(size=6, chars=string.ascii_uppercase):
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    return timestamp + "." + "".join(random.choice(chars) for _ in range(size))


class RandomWrapper(object):
    def __init__(self, seed):
        self.seed = seed
        self.state = None

    def __enter__(self):
        self.state = np.random.get_state()
        np.random.seed(self.seed)
        return self.state

    def __exit__(self, exc_type, exc_val, exc_tb):
        np.random.set_state(self.state)


def get_random_generator(seed):
    return np.random.RandomState(seed)


def set_cpu_random_seed(seed):
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)
