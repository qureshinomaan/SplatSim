from time import strftime, localtime
from collections import defaultdict
from datetime import date
import time


def td_format(td_object):
    seconds = int(td_object.total_seconds())
    periods = [("y", 60 * 60 * 24 * 365), ("m", 60 * 60 * 24 * 30), ("d", 60 * 60 * 24), ("h", 60 * 60), ("m", 60), ("s", 1)]
    ret = ""
    for period_name, period_seconds in periods:
        if seconds > period_seconds:
            period_value, seconds = divmod(seconds, period_seconds)
            ret += f"{period_value}{period_name}"
    return ret


class TimerError(Exception):
    def __init__(self, message):
        self.message = message
        super(TimerError, self).__init__(message)


class Timer:
    """A flexible Timer class.

    :Example:

    >>> import time
    >>> import pyrl
    >>> with pyrl.Timer():
    >>>     # simulate a code block that will run for 1s
    >>>     time.sleep(1)
    1.000
    >>> with pyrl.Timer(print_tmpl='it takes {:.1f} seconds'):
    >>>     # simulate a code block that will run for 1s
    >>>     time.sleep(1)
    it takes 1.0 seconds
    >>> timer = pyrl.Timer()
    >>> time.sleep(0.5)
    >>> print(timer.since_start())
    0.500
    >>> time.sleep(0.5)
    >>> print(timer.since_last_check())
    0.500
    >>> print(timer.since_start())
    1.000
    """

    def __init__(self, start=True, print_tmpl="{:.3f}"):
        self._is_running = False
        self.print_tmpl = print_tmpl
        self._total_time = defaultdict(float)
        if start:
            self.start()

    @property
    def is_running(self):
        """bool: indicate whether the timer is running"""
        return self._is_running

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, type, value, traceback):
        self._is_running = False
        return self.since_start()

    def start(self):
        """Start the timer."""
        if not self._is_running:
            self._t_start = time.time()
            self._is_running = True
        self._t_last = time.time()

    def since_start(self):
        """Total time since the timer is started.
        Returns (float): Time in seconds.
        """
        if not self._is_running:
            raise TimerError("timer is not running")
        self._t_last = time.time()
        return self._t_last - self._t_start

    def since_last_check(self, task_name=None):
        """Time since the last checking.
        Either :func:`since_start` or :func:`since_last_check` is a checking operation.
        Returns (float): Time in seconds.
        """
        if not self._is_running:
            raise TimerError("timer is not running")
        dur = time.time() - self._t_last
        self._t_last = time.time()
        if task_name is not None:
            self._total_time[task_name] += dur
        return dur
    
    def clear_history(self):
        self._total_time = defaultdict(float)
        
    def __getitem__ (self, name: str):
        return self._total_time[name]


global_timers = {}


def check_time(timer_id):
    """Add check points in a single line.
    This method is suitable for running a task on a list of items. A timer will be registered when the method is called
    for the first time.

    :Example:

    >>> import time
    >>> import pyrl
    >>> for i in range(1, 6):
    >>>     # simulate a code block
    >>>     time.sleep(i)
    >>>     pyrl.check_time('task1')
    2.000
    3.000
    4.000
    5.000

    Args:
        timer_id (str): Timer identifier.
    """
    if timer_id not in global_timers:
        global_timers[timer_id] = Timer()
        return 0
    else:
        return global_timers[timer_id].since_last_check()


def get_time_stamp():
    return strftime("%Y%m%d_%H%M%S", localtime())


def get_today():
    return date.today()
