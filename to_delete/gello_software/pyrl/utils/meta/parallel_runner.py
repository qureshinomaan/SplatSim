from inspect import isfunction
from ctypes import c_bool, c_int32
from multiprocessing import Process, set_start_method, Pipe, Value, Semaphore
from copy import deepcopy
import numpy as np, random, time


try:
    set_start_method("forkserver")
except RuntimeError:
    pass


class Worker(Process):
    ASK = 1
    CALL = 2
    GETATTR = 3
    CONTINUE = 4
    EXIT = 5

    def __init__(self, cls, worker_id, worker_seed=None, daemon=True, mem_infos=None, is_class=True, *args, **kwargs):
        super(Process, self).__init__()
        # Set basic information
        self.worker_id = worker_id
        self.worker_seed = worker_seed

        # Set parameters for class or functions
        self.cls = cls
        self.is_class = is_class
        self.args = deepcopy(args)
        self.kwargs = deepcopy(dict(kwargs))
        self.kwargs["worker_id"] = worker_id

        # Set process config
        self.pipe, self.worker_pipe = Pipe(duplex=True)
        self.daemon = daemon

        # Shared memory
        use_shared_memory = mem_infos is not None
        self.initialized = Value(c_bool, False)
        self.running = Value(c_bool, False)
        self.item_in_pipe = Value(c_int32, 0)
        self.shared_memory = Value(c_bool, use_shared_memory)
        self.mem_infos = mem_infos

        if use_shared_memory:
            self.shared_mem_all = None
            self.shared_mem = None
            from multiprocessing import shared_memory
            self.input_mem = shared_memory.SharedMemory(create=True, size=1024**2)  # 1M input information
            self.len_input = Value(c_int32, 0)
            self.lock = Semaphore(0)
        else:
            self.input_mem = None
            self.len_input = None

        if hasattr(self, "start"):
            self.start()
        else:
            print("We should merge this class to another class")
            exit(0)

    def _return_results(self, ret):
        if self.shared_memory.value:
            with self.item_in_pipe.get_lock():
                self.item_in_pipe.value += 1
            # Shared memory needs the object assignment to be finished
        else:
            # Send object will wait the object to be received!
            with self.item_in_pipe.get_lock():
                self.item_in_pipe.value += 1
            self.worker_pipe.send(ret)

    def run(self):
        from pyrl.utils.data import SharedDictArray
        from pyrl.utils.file import load

        if self.shared_memory.value:
            assert self.is_class
            self.shared_mem_all = SharedDictArray(None, *self.mem_infos)
            self.shared_mem = self.shared_mem_all.to_dict_array().slice(self.worker_id)
            func = self.cls(*self.args, **self.kwargs, buffers=self.shared_mem.memory)
        elif self.is_class:
            func = self.cls(*self.args, **self.kwargs)

        if self.worker_seed is not None:
            np.random.seed(self.worker_seed)
            random.seed(self.worker_seed)
            # if self.is_class and hasattr(func, "seed"):
            # For gym environment
            # func.seed(self.worker_seed)

        self.running.value = False
        with self.item_in_pipe.get_lock():
            self.item_in_pipe.value = 0
        while True:
            # Wait for next commands
            self.initialized.value = True
            if self.shared_memory.value:
                self.lock.acquire()
                op, args, kwargs = load(bytes(self.input_mem.buf[: self.len_input.value]), file_format="pkl")
                with self.len_input.get_lock():
                    self.len_input.value = 0

            else:
                op, args, kwargs = self.worker_pipe.recv()

            if op == self.CONTINUE:
                continue

            if op == self.EXIT:
                if func is not None and self.is_class:
                    del func
                self.worker_pipe.close()
                return

            self.running.value = True

            if op == self.ASK:
                ret = func(*args, **kwargs)
            elif op == self.CALL:
                assert self.is_class
                func_name = args[0]
                args = args[1]
                ret = getattr(func, func_name)(*args, **kwargs)
            elif op == self.GETATTR:
                assert self.is_class
                ret = getattr(func, args)

            self.running.value = False
            self._return_results(ret)

    def _send_info(self, info):
        """
        Executing some functions, before this we need to clean up the reaming results in pipe.
        It is important when we use async_get.
        """
        assert self.item_in_pipe.value in [0]
        if bool(self.shared_memory.value):
            from pyrl.utils.file import dump
            info = dump(info, file_format="pkl")
            self.input_mem.buf[: len(info)] = info
            self.len_input.value = len(info)
            self.lock.release()
        else:
            self.pipe.send(info)

    def call(self, func_name, *args, **kwargs):
        self._send_info([self.CALL, [func_name, args], kwargs])

    def get_attr(self, attr_name):
        self._send_info([self.GETATTR, attr_name, None])

    def ask(self, *args, **kwargs):
        self._send_info([self.ASK, args, kwargs])

    @property
    def is_running(self):
        return self.running.value

    @property
    def is_idle(self):
        return not self.running.value and self.item_in_pipe.value == 0

    @property
    def is_ready(self):
        return not self.running.value and self.item_in_pipe.value > 0
    
    @property
    def status(self):
        return {"Running": self.running.value, "Item in process": self.item_in_pipe.value}

    def set_shared_memory(self, value=True):
        if self.shared_memory.value == value:
            return
        self.shared_memory.value = value
        if value:
            # When not shared memory, the sub-process does not use busy waiting. We need to send a signal to the sub-process to make them know that the mode is changed
            self.pipe.send([self.CONTINUE, None, None])
        else:
            from pyrl.utils.file import dump
            info = dump([self.CONTINUE, None, None], file_format="pkl")
            self.input_mem.buf[: len(info)] = info
            self.len_input.value = len(info)
            self.lock.release()

    def wait(self, timeout=-1):
        """
        Wait for sub-process and return its output to main process.
        If the process use shared memory, then return no-thing.
        """
        start_time = None
        while self.item_in_pipe.value == 0 or self.is_running:
            if self.initialized.value and start_time is None:
                start_time = time.time()
            if start_time is not None and time.time() - start_time > timeout and timeout > 0:
                raise RuntimeError(f"Nothing to get from pipe after {time.time() - start_time}s")
        with self.item_in_pipe.get_lock():
            self.item_in_pipe.value -= 1
        return "Received" if self.shared_memory.value else self.pipe.recv()

    def wait_async(self):
        """
        Check the status of the sub-process and return its output to main process if it is finished.
        If the process use shared memory, then return no-thing.
        """
        ret = None
        if self.item_in_pipe.value > 0 and not self.is_running:
            assert self.item_in_pipe.value == 1, f"{self.item_in_pipe.value}"
            ret = "Received" if self.shared_memory.value else self.pipe.recv()
            with self.item_in_pipe.get_lock():
                self.item_in_pipe.value -= 1
        return ret

    def debug_print(self):
        print("Out", self.shared_memory.value, self.item_in_pipe.value, self.running.value)

    def close(self):
        if self.is_alive():
            self.terminate()
        if self.input_mem is not None:
            self.input_mem.unlink()
            self.input_mem.close()
        del self.pipe
        del self.worker_pipe
