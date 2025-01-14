from isaacgym.torch_utils import *
import open3d
from http.cookiejar import Cookie
from sklearn.cluster import linkage_tree
import torch
import numpy as np
import os
import ipdb
import time
import matplotlib.pyplot as plt


def mkdir(path):
	# 引入模块
	import os
 
	# 去除首位空格
	path=path.strip()
	# 去除尾部 \ 符号
	path=path.rstrip("\\")
 
	# 判断路径是否存在
	# 存在	 True
	# 不存在   False
	isExists=os.path.exists(path)
 
	# 判断结果
	if not isExists:
		# 如果不存在则创建目录
		# 创建目录操作函数
		os.makedirs(path) 
		print(path +' 创建成功')
		return True
	else:
		# 如果目录存在则不创建，并提示目录已存在
		print(path+' 目录已存在')
		return False


class CollisionBuffer2Map:
	def __init__(self, obj_path, buffer_path, link_name):
		self.link_name = link_name
		self.obj_path = obj_path
		self.buffer_path = buffer_path + '/2560.pt'
		self.PC_path = obj_path+"/point_sample/full_PC.ply"
		self.PC_path_np = obj_path+"/point_sample/full_PC.npy"
		self.PC_np = np.load(self.PC_path_np)
		self.PC = open3d.io.read_point_cloud(self.PC_path)
		self.relative_buffer = torch.load(self.buffer_path).cpu().numpy()
		self.link_p = np.load(obj_path+"/point_sample/link_p.npy")
		self.link_q = np.load(obj_path+"/point_sample/link_q.npy")
		
		self.other_part_path_np = []
		self.other_part_path_ply = []

		files= os.listdir(obj_path+"/point_sample/")
		for file in files:
			if 'npy' in file:
				if link_name in file:
					self.link_PC_np_path = obj_path+"/point_sample/"+file
				elif 'Actor' in file:
					self.other_part_path_np.append(obj_path+"/point_sample/"+file)
			if 'ply' in file:
				if link_name in file:
					self.link_PC_ply_path = obj_path+"/point_sample/"+file
				elif 'Actor' in file:
					self.other_part_path_ply.append(obj_path+"/point_sample/"+file)

		self.link_np_PC = np.load(self.link_PC_np_path)
		self.link_ply_PC = open3d.io.read_point_cloud(self.link_PC_ply_path)

		solute_buffer = []
		for relative_pose in self.relative_buffer:
			pos = torch.tensor(self.link_p, device=0) + quat_apply(torch.tensor(self.link_q, device=0), 
				torch.tensor(relative_pose[0:3], device=0)).view(-1)
			solute_buffer.append(pos.cpu().numpy())

		self.buffer = np.array(solute_buffer)

	def get_map(self, dis_bar, if_save, save_path=None ):

		link_PC = torch.tensor(self.link_np_PC)
		num_points_around = torch.zeros(link_PC.shape[0])

		for collision_p in self.buffer:
			delta_dis = link_PC-torch.tensor(collision_p)
			num_points_around = num_points_around+(torch.norm(delta_dis, dim=1)<dis_bar)
		max = torch.max(num_points_around)
		if max == 0:
			max = 1e-8
		points_around_scale = (num_points_around*1./(max*1.)).view(num_points_around.shape[0], 1)

		heat_map = torch.cat((link_PC, points_around_scale), dim=1).numpy()
		if if_save:
			np.save(save_path+'part_heat_map.npy', heat_map)
		
		return heat_map, max
			
	def get_other_part_map(self):
		total_map = np.array([])
		for file in self.other_part_path_np:
			part_PC = np.load(file)
			tmp = np.zeros((part_PC.shape[0], 1))
			heat_map = np.concatenate((part_PC, tmp), axis=1)
			if(total_map.shape[0]==0):
				total_map = heat_map
			else:
				total_map = np.concatenate((total_map, heat_map))
		return total_map
				
	def get_full_map(self, if_save=False, save_path=None):
		other_map = self.get_other_part_map()
		heat_map, _ = self.get_map(0.5, False)
		full_map = np.concatenate((heat_map, other_map))
		self.full_map = full_map
		if if_save:
			mkdir(save_path)
			np.save(save_path+'full_map.npy', full_map)
		return full_map

	def visualization(self, if_save=False, save_path=None):
		data = self.full_map[:, :3]
		label = self.full_map[:, 3]
		data=data[:,:3]
		labels=np.asarray(label)

		# 颜色
		colors = plt.get_cmap()(labels)
		pt1 = open3d.geometry.PointCloud()
		pt1.points = open3d.utility.Vector3dVector(data.reshape(-1, 3))
		pt1.colors=open3d.utility.Vector3dVector(colors[:, :3])
		if if_save:
			vis = open3d.visualization.Visualizer()		
			vis.create_window()
			vis.add_geometry(pt1)		
			# vis.update_geometry(point_cloud)
			vis.poll_events()
			vis.update_renderer()
			# image path
			image_path = save_path+'full_map.jpg'
			vis.capture_screen_image(image_path)
			vis.destroy_window()
		else:
			open3d.visualization.draw_geometries([pt1])


if __name__ == '__main__':
	dataset_path = 'assets/dataset/one_door_cabinet'
	buffer_folder_path = 'dexteroushandenvs/runs/cabinet'
	# get all urdf path
	files = os.listdir(dataset_path)
	n_points = 10000
	zero_num = 0
	total_num = 0
	for item in files:
		print("showing", item)
		buffer_path = buffer_folder_path+'/'+item
		obj_path = dataset_path+'/'+item
		link_name = item[6:]
		buffer2map = CollisionBuffer2Map(obj_path, buffer_path, link_name)
		buffer2map.get_full_map(if_save=False, save_path=dataset_path+'/'+item+'/heat_map/')
		buffer2map.visualization(if_save=False, save_path=dataset_path+'/'+item+'/heat_map/')
		# map, max = buffer2map.get_map(0.5,if_save=False,  dataset_path+'/'+item+'/point_sample/')
		# if(max<1):
		# 	zero_num+=1
		# total_num+=1
		# print("Invalid part ratio:", zero_num*1./total_num)


