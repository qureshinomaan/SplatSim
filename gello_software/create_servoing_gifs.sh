#!/bin/bash

# Base directory
BASE_DIR="recordings"

# Directory to store gifs
GIF_DIR="$BASE_DIR/gifs"

# Create the gifs directory
mkdir -p "$GIF_DIR"

# Loop through each subdirectory in the recordings folder
for TARGET_DIR in "$BASE_DIR"/*/; do
    TARGET_NAME=$(basename "$TARGET_DIR")
    GIF_TARGET_DIR="$GIF_DIR/$TARGET_NAME"
    
    # Create corresponding folder in the gifs directory
    mkdir -p "$GIF_TARGET_DIR"

    # Loop through each subdirectory (depth, flow, interaction, rgb)
    for SUB_DIR in "$TARGET_DIR"*/; do
        SUB_DIR_NAME=$(basename "$SUB_DIR")
        IMAGE_PATH="$SUB_DIR/*.png" # Assuming images are .png files
        OUTPUT_GIF="$GIF_TARGET_DIR/$SUB_DIR_NAME.gif"

        # Create the gif using ffmpeg
        if ls $SUB_DIR/*.png 1> /dev/null 2>&1; then
            ffmpeg -y -framerate 10 -pattern_type glob -i "$IMAGE_PATH" -vf "scale=640:-1:flags=lanczos" "$OUTPUT_GIF"
            echo "Created GIF: $OUTPUT_GIF"
        else
            echo "No images found in $SUB_DIR_NAME for $TARGET_NAME"
        fi
    done
done
