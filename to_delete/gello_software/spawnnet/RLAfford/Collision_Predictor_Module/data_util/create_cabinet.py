import os
import pathlib
import xml.etree.ElementTree as ET
import shutil
import yaml

def get_cabinet_name_list(file_name) :

	with open(file_name, "r") as f:
		
		data = f.read().splitlines()

	return data

def build_new_cabinet(src_path, dst_path, link_id) :

	urdf = ET.parse(os.path.join(src_path, 'mobility.urdf'))
	root = urdf.getroot()
	for child in root :
		if child.tag == "joint" :
			name = child.attrib["name"]
			vis = False
			for tmp in child.findall('child') :
				if tmp.attrib['link'] == link_id :
					vis = True
			for tmp in child.findall('parent') :
				if tmp.attrib['link'] == link_id :
					vis = True
			if vis :
				pass
			else :
				# not the joint meant to be revolute
				child.attrib["type"] = "fixed"
	if not os.path.exists(dst_path):
		# 如果目标路径不存在原文件夹的话就创建
		os.makedirs(dst_path)

	if os.path.exists(src_path):
		# 如果目标路径存在原文件夹的话就先删除
		shutil.rmtree(dst_path)

	shutil.copytree(src_path, dst_path)

	urdf.write(os.path.join(dst_path, 'mobility.urdf'), xml_declaration=True)

def process_cabinet(path) :

	name_list = []
	path_list = []
	
	semantic_file = os.path.join(path, "semantics.txt")
	urdf_file = os.path.join(path, "mobility.urdf")
	
	with open(semantic_file, "r") as f:

		semantics = f.read().splitlines()
		for link_repr in semantics :
			link_id, link_type, link_name = link_repr.split(' ')

			if link_type == "hinge" and link_name == "rotation_door":
				# found a rotation door
				new_cabinet_name = os.path.basename(path) + "_"  + link_id
				dst_path = os.path.join("dataset/one_door_cabinet", new_cabinet_name)
				build_new_cabinet(path, dst_path, link_id)
				name_list.append(new_cabinet_name)
				path_list.append(dst_path)
	
	return name_list, path_list

if __name__ == "__main__" :

	root = "./assets"
	cabinet_name_list = get_cabinet_name_list("./cabinet.txt")
	print("total", len(cabinet_name_list), "cabinets")

	name_list = []
	path_list = []

	for name in cabinet_name_list :
		
		new_name_list, new_path_list = process_cabinet(os.path.join(root, name))
		name_list += new_name_list
		path_list += new_path_list
	
	save_dict = {}
	
	for name, path in zip(name_list, path_list) :
		item = {
			"name": name,
			"path": os.path.join(path, "mobility.urdf"),
			"boundingBox": os.path.join(path, "bounding_box.json"),
			"pointCloud": os.path.join(path, "point_sample", "ply-10000.ply")
		}
		save_dict[name] = item
	
	print(save_dict)
	with open("./dataset_conf.yaml", "w") as f:
		print(yaml.dump(save_dict), file=f)