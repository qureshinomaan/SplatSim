import os
import json

def is_cabinet(path) :

	with open(os.path.join(path, 'meta.json'), 'r') as f :
		meta = json.load(f)
	
	if meta["model_cat"] == "StorageFurniture" :
		return True
	else :
		return False

root = "assets"
cabinet_list = []

for s in os.listdir(root) :

	if is_cabinet(os.path.join(root, s)) :
		
		cabinet_list.append(s)
	
with open("cabinet.txt", "w") as f:
	for line in cabinet_list :
		print(line, file=f)
