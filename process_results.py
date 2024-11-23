import os
import cv2
import numpy as np

def get_last_image(directory):
    """Helper function to get the last image from a sorted list of images."""
    files = sorted([f for f in os.listdir(directory) if f.endswith(('.png', '.jpg', '.jpeg'))])
    if files:
        return cv2.imread(os.path.join(directory, files[-1]))
    return None

def average_images(images):
    """Calculate the average of a list of images."""
    if not images:
        return None
    # Stack images along the third axis and take the mean
    stack = np.stack(images, axis=3)
    return np.mean(stack, axis=3).astype(np.uint8)

def process_folders(main_folder):
    """Process subfolders and compute average images for images_1 and images_2."""
    images_1 = []
    images_2 = []

    for folder in os.listdir(main_folder):
        folder_path = os.path.join(main_folder, folder)
        if os.path.isdir(folder_path):
            images_1_path = os.path.join(folder_path, 'images_1')
            images_2_path = os.path.join(folder_path, 'images_2')

            # Get the last image from each subfolder
            last_image_1 = get_last_image(images_1_path)
            last_image_2 = get_last_image(images_2_path)

            if last_image_1 is not None:
                images_1.append(last_image_1)
            if last_image_2 is not None:
                images_2.append(last_image_2)

    # Calculate averages
    average_image_1 = average_images(images_1)
    average_image_2 = average_images(images_2)

    # Save averages in a new directory
    output_dir = os.path.join(os.getcwd(), 'averaged_images')
    os.makedirs(output_dir, exist_ok=True)
    
    if average_image_1 is not None:
        cv2.imwrite(os.path.join(output_dir, 'average_images_1.jpg'), average_image_1)
        print('Average image for images_1 saved successfully.')

    if average_image_2 is not None:
        cv2.imwrite(os.path.join(output_dir, 'average_images_2.jpg'), average_image_2)
        print('Average image for images_2 saved successfully.')

if __name__ == "__main__":
    main_folder = '/home/nomaan/bc_data_160/gello'
    process_folders(main_folder)
