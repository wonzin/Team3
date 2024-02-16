import shutil
import os
import requests
from PIL import Image
from io import BytesIO
import torch
import numpy as np
import cv2
# Path to your model
model_path = './runs/train2/weights/best.pt'
conf_threshold = 0.8
image_url = 'http://172.26.60.68:8001/scotty_real.jpg'
image_path = './scotty_real.jpg'

min_values = torch.tensor([0.1, 0.9, 0.1, 0.9])

# Importing YOLO from ultralytics
from ultralytics import YOLO

# Function to download the image
def download_image(url, path):
    response = requests.get(url)
    if response.status_code == 200:
        with open(path, 'wb') as f:
            f.write(response.content)
    else:
        print("Failed to download the image.")

def calibration(img):
    npz_path = "/home/orin/catkin_ws/src/jetbot_pro/scripts/calibrate_data/intrinsicCalibration.npz"
    dist_arr = np.load(npz_path)
    objpoints = dist_arr["objpoints"]
    imgpoints = dist_arr["imgpoints"]
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img.shape[1:], None, None)
    undist = cv2.undistort(img, mtx, dist, None, mtx)
    return undist

# Initialize the model
model = YOLO(model_path)
model.conf = conf_threshold

with open("scotty.txt", "w") as file:
    file.write('No file')
while True:
    # Check if runs/detect exists, and if so, remove it
    detect_path = './runs/detect'
    if os.path.exists(detect_path):
        shutil.rmtree(detect_path)
    if os.path.exists(image_path):
        os.remove(image_path)
    # Download the image
    download_image(image_url, image_path)

    # Load the downloaded image
    img = Image.open(image_path)

    # Calibration
    cv_img = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    img = calibration(cv_img)


    # Perform detection
    results = model(img, save=True)
    print(results[0].boxes)
    if len(results) > 0 and hasattr(results[0].boxes, 'conf') and results[0].boxes.conf.numel() > 0:
        
        # Safe to proceed
        print(round((results[0].boxes.conf).item(), 6))
    else:
        # Handle the case where the tensor is empty or the structure is not as expected
        print("No boxes found or the confidence score tensor is empty.")
        continue
    # Check if 'scotty' is detected

    xyxyn = results[0].boxes.xyxyn
    print(results[0].boxes.xyxyn)


    #if (results[0].boxes.conf).item() > conf_threshold:
    #    print("Scotty detected!")
    if (results[0].boxes.conf).item() > conf_threshold and ((xyxyn[0, :2] >= 0.1).any() and (xyxyn[0, 2:] <= 0.9).any()).item():
        print("Scotty detected!")
        
        with open("scotty.txt", "w") as file:
            file.write("Scotty")
        
        break  # Exit the loop if 'scotty' is found
    else:
        print("Scotty not detected, retrying...")
        # Optional: Add a delay here if you want to limit the frequency of requests

