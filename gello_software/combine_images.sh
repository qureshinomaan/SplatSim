#!/bin/bash

# Create a new directory to store combined images
mkdir combined_images

# Initialize a counter to keep track of the image index
counter=0

# List of folders in the order you want to combine them
folders=("stream_pos_1" "stream_pos_2" "stream_free_1" "stream_end_effector")

# Loop through each folder
for folder in "${folders[@]}"
do
    echo "Processing $folder..."
    # Ensure the folder exists
    if [[ -d "$folder" ]]; then
        # Get all png files in the current folder, sorted by filename
        files=$(ls $folder/*.png | sort -V)
        for file in $files
        do
            # Construct new filename based on the counter
            new_file=$(printf "image_%d.png" "$counter")
            echo "Copying $file to combined_images/$new_file"
            # Copy file to the new directory with the new name
            cp "$file" "combined_images/$new_file"
            # Increment the counter
            ((counter++))
        done
    else
        echo "Directory $folder does not exist"
    fi
done

echo "All images have been combined."
