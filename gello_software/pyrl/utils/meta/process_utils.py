import psutil, os


def format_memory_str(x, unit, number_only=False):
    unit_list = ["K", "M", "G", "T"]
    assert unit in unit_list
    unit_num = 1024 ** (unit_list.index(unit) + 1)
    if number_only:
        return x * 1.0 / unit_num
    else:
        return f"{x * 1.0 / unit_num:.2f}{unit}"


def get_total_memory(unit="G", number_only=False, init_pid=None):
    from pyrl.utils.data import num_to_str

    if init_pid is None:
        init_pid = os.getpid()
    process = psutil.Process(init_pid)
    ret = process.memory_full_info().uss
    for proc in process.children():
        process_info = proc.memory_full_info()
        ret += process_info.uss
    return num_to_str(ret, unit, number_only=number_only)


def get_memory_list(unit="G", number_only=False, init_pid=None):
    from pyrl.utils.data import num_to_str

    if init_pid is None:
        init_pid = os.getpid()
    process = psutil.Process(init_pid)
    ret = [
        num_to_str(process.memory_full_info().uss, unit, number_only=number_only),
    ]
    for proc in process.children():
        process_info = proc.memory_full_info()
        ret.append(num_to_str(process_info.uss, unit, number_only=number_only))
    return ret


def get_memory_dict(unit="G", number_only=False, init_pid=None):
    from pyrl.utils.data import num_to_str

    if init_pid is None:
        init_pid = os.getpid()
    process = psutil.Process(init_pid)
    ret = {init_pid: num_to_str(process.memory_full_info().uss, unit, number_only=number_only)}
    for i, proc in enumerate(process.children()):
        process_info = proc.memory_full_info()
        ret[proc.pid] = num_to_str(process_info.uss, unit, number_only=number_only)
    return ret


def get_subprocess_ids(init_pid=None):
    if init_pid is None:
        init_pid = os.getpid()
    ret = [init_pid]
    process = psutil.Process(os.getpid())
    for proc in process.children():
        ret.append(proc.pid)
    return ret
