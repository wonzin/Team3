import shutil
import os
import requests
from PIL import Image
from io import BytesIO
import torch
import cv2
import numpy as np
from ultralytics.utils import ASSETS, yaml_load
from PIL import Image, ImageDraw, ImageFont

# Path to your model
# model_path = './runs/train2/weights/best.pt'
model_path = './yolov8m.pt'
# model_path = './yolov8s.pt'
conf_threshold = 0.8
image_url = 'http://172.26.60.68:8123/jetbot_object_detection.png'
image_path = './jetbot_object_detection.png'

text_url = 'http://172.26.60.68:8123/jetbot_coord.txt'
text_path = './jetbot_coord.txt'

bomb_url = 'http://172.26.60.68:8123/bomb_coord.txt'
bomb_path = './bomb_coord.txt'

bounding_box_path= './bounding_box.png'

CLASSES = yaml_load("/home/orin/catkin_ws/src/jetbot_pro/scripts/yolov8/coco128.yaml")["names"]
colors = np.random.uniform(0, 255, size=(len(CLASSES), 3))
min_values = torch.tensor([0.1, 0.9, 0.1, 0.9])
# Importing YOLO from ultralytics
from ultralytics import YOLO

def draw_bounding_box(img, class_name, confidence, x, y, x_plus_w, y_plus_h):
    """
    Draws bounding boxes on the input image based on the provided arguments.

    Args:
        img (numpy.ndarray): The input image to draw the bounding box on.
        class_id (int): Class ID of the detected object.
        confidence (float): Confidence score of the detected object.
        x (int): X-coordinate of the top-left corner of the bounding box.
        y (int): Y-coordinate of the top-left corner of the bounding box.
        x_plus_w (int): X-coordinate of the bottom-right corner of the bounding box.
        y_plus_h (int): Y-coordinate of the bottom-right corner of the bounding box.
    """
    label = f"{CLASSES[class_id]} ({confidence:.2f})"
    # class_id = int(class_id)
    # print("here class id", class_id)
    fnt = ImageFont.truetype("Pillow/Tests/fonts/FreeMono.ttf", 15)
    shape = [x, x_plus_w, y, y_plus_h]
    img1 = ImageDraw.Draw(img)
    img1.rectangle(shape, outline ="red")
    tmp_str = class_name + " " + str(confidence)
    img1.text((0, 0), tmp_str, font=fnt, fill=(0, 0, 0, 0), align='center')
    img.save(bounding_box_path)
    # img = np.array(img)
    # print("img: ", img.shape)
    #color = colors[int(class_id)]
    #cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)
    #cv2.putText(img, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    #cv2.imwrite(bounding_box_path, img)

# Function to download the image or text content
def download_content(url, path):
    response = requests.get(url)
    if response.status_code == 200:
        with open(path, 'wb') as f:
            f.write(response.content)
    else:
        print("Failed to download the content of "+url+" "+path)

def calibration(img):
    npz_path = "/home/orin/catkin_ws/src/jetbot_pro/scripts/calibrate_data/intrinsicCalibration.npz"
    dist_arr = np.load(npz_path)
    objpoints = dist_arr["objpoints"]
    imgpoints = dist_arr["imgpoints"]
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img.shape[1:], None, None)
    undist = cv2.undistort(img, mtx, dist, None, mtx)
    pil_undist = Image.fromarray(undist)
    return pil_undist

# Initialize the model
model = YOLO(model_path)
model.conf = conf_threshold

with open("bool_detection.txt", "w") as file:
    file.write('0')

# today_bomb = "bottle"
# today_bomb = "backpack"
today_bomb = "jimin"
detection = False

while True:
    # Check if runs/detect exists, and if so, remove it
    detect_path = './runs/detect'
    if os.path.exists(detect_path):
        shutil.rmtree(detect_path)
    if os.path.exists(image_path):
        os.remove(image_path)
    # Download the image
    download_content(image_url, image_path)

    # Download coordinate text
    download_content(text_url, text_path)

    # Download bomb coord text
    # download_content(bomb_url, bomb_path)

    # Load the downloaded image
    img = Image.open(image_path)

    # Calibration
    cv_img = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    img = calibration(cv_img)
   
    with open("jetbot_object_detection.txt", "w") as file:
        file.write(str(False)+'\n')

    # Perform detection
    results = model(img, save=True)
    class_list = results[0].names
    # print("class_list: ", class_list)
    # print("results: ", results)
    if len(results) > 0 and hasattr(results[0].boxes, 'conf') and results[0].boxes.conf.numel() > 0:
        # Safe to proceed
        for idx, cl in enumerate(results[0].boxes.cls):
            class_id = cl.item()
            print(class_list[class_id], " object found. confidence score: ", round((results[0].boxes.conf[idx]).item(), 6))
            #with open("jetbot_object_detection.txt", "a") as file:
            #    file.write(str(class_list[class_id])+' '+str(round((results[0].boxes.conf[idx]).item(), 6))+'\n')
            # if class_list[class_id] == today_bomb or class_list[class_id] == 'backpack' or class_list[class_id] == 'person' or class_list[class_id] == 'chair':
            if class_list[class_id] == today_bomb:
                print("detection True " + class_list[class_id])
                bounding_box_coord = results[0].boxes.xyxy[0] # bounding box coordinate
                print("bounding_box_coord: ", bounding_box_coord)
                x, x_plus_w, y, y_plus_h = bounding_box_coord[0], bounding_box_coord[1], bounding_box_coord[2], bounding_box_coord[3]
                conf = round((results[0].boxes.conf[idx]).item(), 2)
                class_name = class_list[class_id]
                draw_bounding_box(img, class_name, conf, x, y, x_plus_w, y_plus_h)
                detection = True
                with open("jetbot_object_detection.txt", "w") as file:
                    file.write(str(True)+'\n')
    else:
        # Handle the case where the tensor is empty or the structure is not as expected
        print("No boxes found or the confidence score tensor is empty.")
        continue
    # Check if today's bomb is detected

    if detection == False:
        with open("jetbot_object_detection.txt", "a") as file:
            file.write(str(False)+'\n')

    with open("bool_detection.txt", "w") as file:
        if detection == True:
            value_to_write = 2
        else:
            value_to_write = 0
        # value_to_write = 2 if detection else 1
        file.write(str(value_to_write))

    xyxyn = results[0].boxes.xyxyn
    print("xyxyn: ", results[0].boxes.xyxyn)


    #if (results[0].boxes.conf).item() > conf_threshold:
    #    print("Scotty detected!")
    
    if detection == True and (results[0].boxes.conf[0]).item() > conf_threshold:#  and ((xyxyn[0, :2] >= 0.1).all() and (xyxyn[0, 2:] <= 0.9).all()).item():
        print("Object detected!")
        #with open("bool_detection.txt", "w") as file:
        #    value_to_write = 2 if detection else 1
        #    file.write(str(value_to_write))
        
        # with open("jetbot_object_detection.txt", "w") as file:
        #    file.write("jetbot object detected")
        
        break  # Exit the loop if object is found
    else:
        print("Object not detected, retrying...")
        # Optional: Add a delay here if you want to limit the frequency of requests
    
