import zarr
import numpy as np
import pickle
import os
from numcodecs import Blosc
import cv2

def load_pkl(pkl_file_path):
    with open(pkl_file_path, "rb") as f:
        data = pickle.load(f)
        # state = data["joint_positions"]
        # action = data["control"]
        return data
    
        
def process_folder(folder_path, output_file, compressor):
    # Create zarr file
    z = zarr.open(output_file, mode ='w')
    
    # Create 'data' and 'meta' groups
    data_group = z.create_group('data')
    meta_group = z.create_group('meta')
    # meta_group = meta_group.create_group('episode_ends')    
    
    initial_shape_action = (1, 2)
    initial_shape_state = (1, 2)
    episode_end_arr = []
    episode_idx = 0

    # iterate over folders in the data folder which contain pickle files
    folders = os.listdir(folder_path)

    #if videos folder does not exist, create it
    video_folder_name = "videos"
    videos_folder = os.path.join(video_folder_name)
    if not os.path.exists(videos_folder):
        os.makedirs(videos_folder)
    
    for k in range(len(folders)):
    # for k in range(70):
        print('k :', k)
    # for k in range(1): 
        folder = folders[k]
        folder_pkl = os.path.join(folder_path, folder)
        if not os.path.isdir(folder_pkl):
            continue
        
        allfiles = [file for file in os.listdir(folder_pkl) if file.endswith(".pkl")]
        allfiles.sort()
        img_folder = os.path.join(folder_pkl, "images_1")
        img_files = [file for file in os.listdir(img_folder) if file.endswith(".png")]
        img_files.sort()

        #create folder of name k inside videos folder
        videos_folder = os.path.join(video_folder_name, str(k))
        if not os.path.exists(videos_folder):
            os.makedirs(videos_folder)


        ## command ffmpeg -framerate 24 -i /ocean/projects/agr240001p/mqureshi/corl_data/t_push/gello/0510_025057/images/%05d.png -vf "scale=320:240" -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p 1.mp4
        ## to convert images to video

        os.system("ffmpeg -framerate 80 -i " + img_folder + "/%05d.png -vf \"scale=640:480\" -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p " + videos_folder + "/1.mp4")

        img_folder = os.path.join(folder_pkl, "images_2")
        img_files = [file for file in os.listdir(img_folder) if file.endswith(".png")]
        img_files.sort()

        #create folder of name k inside videos folder
        videos_folder = os.path.join(video_folder_name, str(k))
        if not os.path.exists(videos_folder):
            os.makedirs(videos_folder)


        ## command ffmpeg -framerate 24 -i /ocean/projects/agr240001p/mqureshi/corl_data/t_push/gello/0510_025057/images/%05d.png -vf "scale=320:240" -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p 1.mp4
        ## to convert images to video

        os.system("ffmpeg -framerate 80 -i " + img_folder + "/%05d.png -vf \"scale=640:480\" -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p " + videos_folder + "/2.mp4")


if __name__ == "__main__":
    data_folder = "/home/nomaan/bc_data/policy6DOF/"
    output_folder = "replay_buffer.zarr"
    compressor = Blosc(cname='zstd', clevel=3, shuffle=Blosc.BITSHUFFLE)
    
    process_folder(data_folder, output_folder, compressor)
    print(f"Converted {data_folder} to {output_folder}")