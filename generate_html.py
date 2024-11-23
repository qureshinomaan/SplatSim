import os

def generate_html(folder_path, output_file):
    """Generate HTML file to display videos from given folder path."""
    html_content = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Video Gallery</title>
    <style>
        body { font-family: Arial, sans-serif; padding: 20px; }
        .video-wrapper { margin-bottom: 20px; }
        video { width: 100%; max-width: 640px; height: auto; }
    </style>
</head>
<body>
    <h1>Video Gallery</h1>
    '''

    # Define video MIME types
    mime_types = {
        '.mp4': 'video/mp4',
        '.avi': 'video/x-msvideo',
        '.mov': 'video/quicktime'
    }

    # Scan each subfolder and each file
    for root, dirs, files in os.walk(folder_path):
        print(f"Scanning {root}...")
        for file in files:
            file_ext = os.path.splitext(file)[1]
            if file_ext in mime_types:
                video_path = os.path.join(root, file)
                relative_path = os.path.relpath(video_path, start=os.path.dirname(folder_path))
                html_content += f'''
    <div class="video-wrapper">
        <h2>{file}</h2>
        <video controls>
            <source src="{relative_path}" type="{mime_types[file_ext]}">
            Your browser does not support the video tag.
        </video>
    </div>
    '''

    html_content += '''
</body>
</html>
    '''

    # Write the HTML content to the output file
    with open(output_file, 'w') as f:
        f.write(html_content)
    print(f"HTML page created successfully at {output_file}")

if __name__ == "__main__":
    # Set the path to your videos folder
    videos_folder_path = './videos1'  # Modify this path accordingly
    output_html_file = 'video_gallery_1.html'
    generate_html(videos_folder_path, output_html_file)
   
