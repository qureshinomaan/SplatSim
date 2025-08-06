import pandas, os.path as osp, re, pathlib, os


def load_tb_summaries_as_df(tb_files, exp_names, data_title="test"):
    from tensorboard.backend.event_processing import event_accumulator

    if isinstance(tb_files, (list, tuple)):
        return [load_tb_summaries_as_df(tb_file, exp_name, data_title) for tb_file, exp_name in zip(tb_files, exp_names)]
    else:
        event = event_accumulator.EventAccumulator(tb_files)
        event.Reload()
        # df = pandas.DataFrame()
        exp_begin_time = event.FirstEventTimestamp()
        res = {"exp_name": exp_names}
        running_time = -1
        step = None

        for key in event.scalars.Keys():
            if key.startswith(data_title):
                content = event.scalars.Items(key)
                # print(len(content))
                hours = (content[-1].wall_time - exp_begin_time) / 3600
                if hours > running_time:
                    running_time = hours
                if step is None:
                    step = content[-1].step
                    res["steps"] = [_.step for _ in content]
                if step != content[-1].step:
                    print(f"Step is not consistent: {key}, old {step}, new {content.step}")
                new_key = re.sub(r"\W+", "", key[len(data_title) :])
                res[new_key] = [_.value for _ in content]
        res["running_time"] = running_time
        # df = df.append(res, ignore_index=True)
        return res


def parse_tb_for_rl_exp(path, output_name, data_title="test"):
    """
    Folder structure: Env_name/tf_logs/*tfevents*
    """
    path = pathlib.Path(path)
    tb_files = []
    exp_names = []
    os.makedirs(osp.dirname(output_name), exist_ok=True)
    for env_name in path.glob("*"):
        if env_name.is_dir():
            print(f"Find env {env_name.name}")
            exp_names.append(env_name.name)
            events = list(env_name.glob("*/**/*.tfevents.*"))
            if len(events) > 1:
                print(f"There are mote than one tensorbaord logs in the folder: {str(env_name)}")
            tb_files.append(str(events[0]))
    df = load_tb_summaries_as_df(tb_files, exp_names, data_title)
    print(f"Save log to {osp.abspath(output_name)}")
    df.to_csv(output_name, float_format="%.4f")


def parse_tb_for_rl_alg(path, output_dir="csv_logs", data_title="test"):
    """
    Folder structure: Alogrithm_name/Env_name/tf_logs/*tfevents*
    """
    path = pathlib.Path(path)
    for alg_name in path.glob("*"):
        if alg_name.is_dir():
            print(f"Find alg {alg_name.name}")
            parse_tb_for_rl_exp(str(alg_name), osp.join(output_dir, f"{alg_name.name}.csv"), data_title)
