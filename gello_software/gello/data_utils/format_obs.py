import datetime
import pickle
from pathlib import Path
from typing import Dict

import numpy as np
import cv2


def save_frame(
    folder: Path,
    timestamp: datetime.datetime,
    obs: Dict[str, np.ndarray],
    action: np.ndarray,
) -> None:
    obs["control"] = action  # add action to obs

    # make folder if it doesn't exist
    folder.mkdir(exist_ok=True, parents=True)
    recorded_file = folder / (timestamp.isoformat() + ".pkl")

    # if wrist_rgb in obs, save it as an image in a folder/image
    if "wrist_rgb" in obs.keys():
        image_folder = folder / "images_1"
        image_folder.mkdir(exist_ok=True, parents=True)
        #find the number of images in the folder
        len_images = len(list(image_folder.glob("*.png")))
        #format the image file name as 00000.png
        image_file = image_folder / (str(len_images).zfill(5) + ".png")
        # image_file = image_folder / (timestamp.isoformat() + ".png")
        cv2.imwrite(str(image_file), cv2.cvtColor(obs["wrist_rgb"], cv2.COLOR_RGB2BGR))
        # remove wrist_rgb from obs
        obs.pop("wrist_rgb")

    if "base_rgb" in obs.keys():
        image_folder = folder / "images_2"
        image_folder.mkdir(exist_ok=True, parents=True)
        #find the number of images in the folder
        len_images = len(list(image_folder.glob("*.png")))
        #format the image file name as 00000.png
        image_file = image_folder / (str(len_images).zfill(5) + ".png")
        # image_file = image_folder / (timestamp.isoformat() + ".png")
        cv2.imwrite(str(image_file), cv2.cvtColor(obs["base_rgb"], cv2.COLOR_RGB2BGR))
        # remove base_rgb from obs
        obs.pop("base_rgb")

    with open(recorded_file, "wb") as f:
        pickle.dump(obs, f)
