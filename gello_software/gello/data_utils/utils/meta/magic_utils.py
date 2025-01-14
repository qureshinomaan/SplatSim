# https://github.com/alexmojaki/sorcery
from sorcery import assigned_names, unpack_keys, unpack_attrs, dict_of, print_args, call_with_name, delegate_to_attr, maybe, select_from
from contextlib import contextmanager
from inspect import getfullargspec

# https://github.com/gruns/icecream
# from icecream import ic


def colored_print(output_string, level="info", logger=print):
    from termcolor import colored
    import sys

    if level.lower() in ["warning", "error"]:
        level = colored(level.upper(), "red")
        output_string = colored(output_string, "cyan")
        logger(f"{level}: {output_string}")
    else:
        logger(output_string)


def empty_print(*args, **kwargs):
    pass


def custom_assert(pause, output_string, logger=None):
    if logger is not None:
        logger = logger.log
    else:
        logger = print
    import sys

    if not pause:
        from termcolor import colored
        file_name = colored(sys._getframe().f_code.co_filename, "red")
        line_number = colored(sys._getframe().f_back.f_lineno, "cyan")
        output_string = colored(output_string, "red")
        logger(f"Assert Error at {file_name}, line {line_number}")
        logger(f"Output: {output_string}")


class SlicePrinter:
    def __getitem__(self, index):
        print(index)


slice_printer = SlicePrinter()


@contextmanager
def empty_context_manager():
    yield


def save__init__args(values, underscore=False, overwrite=False, subclass_only=False):
    # Copy from https://github.com/astooke/rlpyt/blob/f04f23db1eb7b5915d88401fca67869968a07a37/rlpyt/utils/quick_args.py#L5
    """
    Use in `__init__()` only; assign all args/kwargs to instance attributes.
    To maintain precedence of args provided to subclasses, call this in the
    subclass before `super().__init__()` if `save__init__args()` also appears
    in base class, or use `overwrite=True`.  With `subclass_only==True`, only
    args/kwargs listed in current subclass apply.
    """
    prefix = "_" if underscore else ""
    self = values["self"]
    args = list()
    Classes = type(self).mro()
    if subclass_only:
        Classes = Classes[:1]
    for Cls in Classes:  # class inheritances
        if "__init__" in vars(Cls):
            args += getfullargspec(Cls.__init__).args[1:]
    for arg in args:
        attr = prefix + arg
        if arg in values and (not hasattr(self, attr) or overwrite):
            setattr(self, attr, values[arg])
