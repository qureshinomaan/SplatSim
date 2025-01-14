#!/bin/bash

# Define the step size k
k=8  # Change this value to whatever k you want

# Create a new directory to store the reduced set of combined images
mkdir reduced_combined_images

# Initialize a counter to keep track of the image index
counter=0

# Get all png files in the combined_images folder, sorted by filename
files=$(ls combined_images/*.png | sort -V)

# Loop through each file
for file in $files
do
    # Check if the current index is a multiple of k
    if (( counter % k == 0 ))
    then
        # Extract the filename from the path
        filename=$(basename "$file")
        echo "Copying $file to reduced_combined_images/$filename"
        # Copy file to the new directory
        cp "$file" "reduced_combined_images/$filename"
    fi
    # Increment the counter
    ((counter++))
done

echo "Selected images have been copied to reduced_combined_images."
