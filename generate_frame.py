import cv2
import os

# Path to the input video
video_path = "/home/thwu/Downloads/assets/open_box.mp4"

# Path to save the extracted frames
output_dir = "/home/thwu/Downloads/assets/open_box_10"
os.makedirs(output_dir, exist_ok=True)  # Create the output directory if it doesn't exist

# Open the video file
cap = cv2.VideoCapture(video_path)

# Check if the video file was successfully opened
if not cap.isOpened():
    print("Failed to open the video file.")
    exit()

# Get the video's frame rate (fps)
original_fps = int(cap.get(cv2.CAP_PROP_FPS))
print(f"Original FPS: {original_fps}")

# Calculate the frame skip interval to achieve 10 Hz
frame_skip_interval = original_fps // 10
print(f"Frame skip interval: {frame_skip_interval}")

frame_count = 0  # Initialize frame counter
saved_frame_count = 0  # Counter for frames actually saved

# Loop through all frames in the video
while True:
    ret, frame = cap.read()  # Read the next frame
    if not ret:
        break  # Exit the loop if there are no more frames

    # Save every `frame_skip_interval`th frame
    if frame_count % frame_skip_interval == 0:
        frame_filename = os.path.join(output_dir, f"frame_{saved_frame_count:06d}.jpg")
        cv2.imwrite(frame_filename, frame)
        saved_frame_count += 1  # Increment the saved frame counter

    frame_count += 1  # Increment the frame counter

# Release the video capture object
cap.release()

print(f"Extraction complete. A total of {saved_frame_count} frames were saved at 10 Hz.")
print(f"Frames are saved in: {output_dir}")
