from PIL import Image
import os
import numpy as np
import imageio

images_path = "output/vineyard_1/train/ours_30000/renders/"

# Create a list of all the image files in the directory
# and read them into a list of PIL images
images = []

for i in range(265):
    image = Image.open(images_path + "image_" + str(i) + ".png")
    images.append(image)