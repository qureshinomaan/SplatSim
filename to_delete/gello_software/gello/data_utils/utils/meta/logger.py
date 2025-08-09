from glob import glob
import os, io
import logging, sys
from .env_var import get_world_rank, get_world_size, is_debug_mode
from collections import OrderedDict


logger_initialized = OrderedDict()


class CustomFormatter(logging.Formatter):

    grey = "\x1b[38;21m"
    yellow = "\x1b[33;21m"
    red = "\x1b[31;21m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format = f"%(name)s - (%(filename)s:%(lineno)d) - %(levelname)s - %(asctime)s - %(message)s"

    FORMATS = {
        logging.DEBUG: grey + format + reset,
        logging.INFO: grey + format + reset,
        logging.WARNING: yellow + format + reset,
        logging.ERROR: red + format + reset,
        logging.CRITICAL: bold_red + format + reset,
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt, datefmt="%Y-%m-%d,%H:%M:%S")
        return formatter.format(record)


def get_logger_name():
    if "PYRL_LOGGER_NAME" not in os.environ:
        return "pyrl"
    if get_world_size() == 1:
        return os.environ["PYRL_LOGGER_NAME"]
    else:
        return os.environ["PYRL_LOGGER_NAME"] + f"-{get_world_rank()}"


def get_logger(name=None, with_stream=True, log_file=None, log_level=logging.INFO):
    """Initialize and get a logger by name.
    If the logger has not been initialized, this method will initialize the logger by adding one or two handlers,
    otherwise the initialized logger will be directly returned. During initialization, a StreamHandler will always be
    added. If `log_file` is specified and the process rank is 0, a FileHandler will also be added.

    Args:
        name (str): Logger name.
        log_file (str | None): The log filename. If specified, a FileHandler will be added to the logger.
        log_level (int): The logger level. Note that only the process of rank 0 is affected, and other processes will
            set the level to "Error" thus be silent most of the time.
    Returns:
        logging.Logger: The expected logger.
    """
    if name is None:
        name = get_logger_name()
    if len(logger_initialized) == 0:
        logging.basicConfig(level=logging.ERROR, handlers=[])

    logger = logging.getLogger(name)
    # e.g., logger "a" is initialized, then logger "a.b" will skip the initialization since it is a child of "a".
    for logger_name, logger_level in logger_initialized.items():
        if name.startswith(logger_name):
            logger.setLevel(logger_level)
            return logger

    logger.propagate = False
    handlers = []

    if with_stream:
        handlers.append(logging.StreamHandler())

    rank = get_world_rank()
    if rank == 0 and log_file is not None:
        file_handler = logging.FileHandler(log_file, "w")
        handlers.append(file_handler)

    formatter = CustomFormatter(datefmt="%Y-%m-%d %H:%M")
    log_fmt = f"%(name)s - (%(filename)s:%(lineno)d) - %(levelname)s - %(asctime)s - %(message)s"
    file_formatter = logging.Formatter(log_fmt, datefmt="%Y-%m-%d,%H:%M:%S")

    logger.handlers = []

    if not (rank == 0 or is_debug_mode()) or not with_stream:
        log_level = logging.ERROR
    
    # exit(0)
    for handler in handlers:
        if isinstance(handler, logging.FileHandler):
            handler.setFormatter(file_formatter)
            logger.addHandler(handler)
        else:
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        
    logger.setLevel(log_level)
    logger_initialized[name] = log_level
    return logger


def print_log(msg, logger="print", level=logging.INFO):
    """Print a log message.
    Args:
        msg (str): The message to be logged.
        logger (logging.Logger | str | None): The logger to be used.
            Some special loggers are:
            - "silent": no message will be printed.
            - other str: the logger obtained with `get_logger(logger)`.
            - None: The `print()` method will be used to print log messages.
        level (int): Logging level. Only available when `logger` is a Logger object or "root".
    """
    if logger == "print":
        flush_print(msg)
    elif isinstance(logger, logging.Logger):
        logger.log(level, msg)
    elif logger == "silent":
        pass
    elif isinstance(logger, str) or logger is None:
        get_logger(logger).log(level, msg)
    else:
        raise TypeError(f'logger should be either a logging.Logger object, str, "silent" or None, ' f"but got {type(logger)}")


def flush_print(*args):
    print(*args)
    sys.stdout.flush()


def flush_logger(logger):
    for h in logger.handlerList:
        h.flush()


class TqdmToLogger(io.StringIO):
    """
    Output stream for TQDM which will output to logger module instead of
    the StdOut.
    """

    logger = None
    level = None
    buf = ""

    def __init__(self, logger=None, level=None):
        super(TqdmToLogger, self).__init__()
        self.logger = logger or get_logger()
        self.level = level or logging.INFO

    def write(self, buf):
        self.buf = buf.strip("\r\n\t ")

    def flush(self):
        self.logger.log(self.level, self.buf)
