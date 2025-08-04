import subprocess, os, time


def collect_env_variables():
    # construct minimal environment
    env = {}
    for k in ["SYSTEMROOT", "PATH", "HOME"]:
        v = os.environ.get(k)
        if v is not None:
            env[k] = v
    # LANGUAGE is used on win32
    env["LANGUAGE"] = "C"
    env["LANG"] = "C"
    env["LC_ALL"] = "C"
    return env


def run_command(command, max_try=1, debug=False):
    env = collect_env_variables()
    ret = None
    for i in range(max_try):
        try:
            ret = subprocess.check_output(command, shell=True, env=env)
            if isinstance(ret, bytes):
                ret = ret.decode()
            break
        except subprocess.CalledProcessError:
            if debug:
                print(f"Try {i}: Subprocess error")
            if i != max_try - 1:
                time.sleep(1)
    if ret is None and debug:
        print(f"Cannot execute the command {command}")
    return ret
