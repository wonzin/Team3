import shutil
import os
import requests
from PIL import Image
from io import BytesIO
import torch
import numpy as np
import cv2


image_url = 'http://172.26.60.68:8004/jetbot_coord.txt'
image_path = './jetbot_coord.txt'

def download_image(url, path):
    response = requests.get(url)
    if response.status_code == 200:
        with open(path, 'wb') as f:
            f.write(response.content)
        print("saving text")
    else:
        print("Failed to download the text.")

while True:
    download_image(image_url,image_path)
