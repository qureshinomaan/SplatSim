import aim, os.path as osp, numpy as np, os
import matplotlib.pyplot as plt
from .builder import EXP_LOGGER
from .base_logger import BaseLogger
from ..meta import get_logger, in_directory


@EXP_LOGGER.register_module(name="aim")
@EXP_LOGGER.register_module()
class AimLogger(BaseLogger):
    """
    experiment: the name of the experiment
    run_hash: uid for resume
    """
    def __init__(self, work_dir: str, log_dir: str, clean_up=False, log_system_params=True, timestamp=None, **kwargs):
        super().__init__(work_dir, timestamp)
        kwargs = dict(kwargs)
        # print(log_dir)
        # exit(0)
        if not osp.exists(osp.join(log_dir, '.aim')):
            os.system(f"cd {log_dir} && aim init")
        name = kwargs["name"] if "name" in kwargs else osp.abspath(work_dir, log_dir) if in_directory(work_dir, log_dir) else work_dir
        if clean_up:
            repo = aim.Repo.from_path(log_dir)
            if name is not None:            
                hashes = []
                for run in repo.query_metrics().iter_runs():
                    if run.run.name == name:
                        hashes.append(run.run.hash)
                repo.delete_runs(hashes)
                if len(hashes) > 0:
                    get_logger().info(f"Delete {len(hashes)} in aim repo {log_dir}!")
        if timestamp is not None and name is not None:
            # Add system assigned timestamp to the experiment's name
            name = name + '-' + timestamp
                    
        self.run = aim.Run(repo=log_dir, log_system_params=log_system_params, **kwargs)
        if name is not None:
            self.run.name = name
        self.run["work_dir"] = osp.abspath(work_dir)

    def log(self, tags, n_iter, tag_name="train"):
        tags = self.get_loggable_tags(tags, tag_name=tag_name)
        self.log_to_csv(tags, n_iter)
        for tag, val in tags.items():
            if np.isscalar(val) or val.size == 1 or isinstance(val, str):
                ret_dict = {"name": tag, "value": val, "step": n_iter, "context":{"subset": tag.split("/")[0]}}
            elif isinstance(val,np.ndarray) and val.ndim==1:
                ret_dict = {"name": tag, "value": np.linalg.norm(val), "step": n_iter, "context":{"subset": tag.split("/")[0]}} 
            else:
                if val.ndim == 2:
                    cmap = plt.get_cmap('jet')
                    val = cmap(val)[..., :3]
                assert val.ndim == 3, f"Image should have three dimension! You provide: {tag, val.shape}!"
                image = aim.Image(val)
                ret_dict= {"name": tag, "value": image, "step": n_iter, "context":{"subset": tag.split("/")[0]}}
            self.run.track(**ret_dict)
            