import numpy as np
import torch
import argparse

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('path', type=str)


args = parser.parse_args()
path = "./"+args.path
print(path)
pc = np.load(path)
pc_torch = torch.from_numpy(pc)
# torch.save()
path = path.replace("model_meshlabserver_normalized_v_pc.npy", "pointcloud_tensor")
torch.save(pc_torch, path)