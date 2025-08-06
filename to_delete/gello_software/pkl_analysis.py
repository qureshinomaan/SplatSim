import pickle
import numpy as np
import matplotlib.pyplot as plt
import cv2
import os

# Load the data
# pkl_file_path = "/media/uas-laptop/umass_1/data_gello/bc_data/gello/0418_180900/2024-04-18T18:09:00.783554.pkl"

# with open(pkl_file_path, "rb") as f:
#     data = pickle.load(f)
    
    
# for key in data.keys():
#     print(key)
#     print(data[key].shape)

all_pkl_files = "/media/uas-laptop/umass_1/data_gello/bc_data/gello"

# search each folder in the directory starting with "04"
k =0 
for folder in os.listdir(all_pkl_files):
    if folder.startswith("04"):
        print(folder)
        for file in os.listdir(os.path.join(all_pkl_files, folder)):
            if file.endswith(".pkl"):
                print(file)
                pkl_file_path = os.path.join(all_pkl_files, folder, file)
                with open(pkl_file_path, "rb") as f:
                    data = pickle.load(f)
                    
                for key in data.keys():
                    print(key)
                    print(data[key].shape)
                    # extract the images and make a video
                    if key == "wrist_rgb":
                        print("wrist_rgb")
                        print(data[key].shape)
                        # make a video+
                        out = cv2.VideoWriter(all_pkl_files + "/_conv/wrist_rgb_"+str(k)+".mp4", cv2.VideoWriter_fourcc(*'XVID'), 30, (640, 480))
                        for i in range(data[key].shape[0]):
                            out.write(data[key][i])
                        out.release()
                        
                    # if key == "wrist_depth":
                    #     print("wrist_depth")
                    #     print(data[key].shape)
                    #     # make a video
                    #     out = cv2.VideoWriter("wrist_depth.avi", cv2.VideoWriter_fourcc(*'XVID'), 30, (640, 480))
                    #     for i in range(data[key].shape[0]):
                    #         out.write(data[key][i])
                    #     out.release()
                        
            k+=1