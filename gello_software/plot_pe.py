import cv2
import os
import numpy as np
import matplotlib.pyplot as plt

# Function to calculate photometric error (mean absolute difference)
def calculate_photometric_error(target, img):
    # Ensure the target and img have the same shape
    if target.shape != img.shape:
        raise ValueError("Images must have the same dimensions.")
    
    # Photometric error: mean squared difference
    pe = np.sum((target.astype(np.float32) - img.astype(np.float32))**2) / float(target.shape[0] * target.shape[1])
    return pe

# Load target image
target_img = cv2.imread('images/target.png', cv2.IMREAD_COLOR)

if target_img is None:
    raise FileNotFoundError("Target image 'target.png' not found.")

# Folder containing the images
folder_path = '/home/nomaan/bc_data/servoing/1021_134523/images_2/'


# Store photometric errors for plotting
errors = []
image_indices = []

# Iterate through images in the folder
for i in range(10000):  # assuming max 9999 images
    img_path = os.path.join(folder_path, f"{i:05d}.png")
    if not os.path.exists(img_path):
        continue
    
    # Load image
    img = cv2.imread(img_path, cv2.IMREAD_COLOR)
    if img is None:
        continue
    
    # Calculate photometric error between target and current image
    pe = calculate_photometric_error(target_img, img)
    errors.append(pe)
    image_indices.append(i)

# Plot the photometric errors
plt.figure(figsize=(10, 6))
plt.plot(image_indices, errors, marker='o', linestyle='-')
plt.title('Photometric Error between Target Image and Image Sequence')
plt.xlabel('Image Index')
plt.ylabel('Photometric Error')
plt.grid(True)
plt.tight_layout()

# Save the plot
plt.savefig('photometric_error_plot.png')
plt.show()
