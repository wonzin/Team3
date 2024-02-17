#!/usr/bin/env python
# encoding: utf-8
import cv2
import numpy as np
import yaml

def find_first_edge_pixel(image, start, direction):
    y, x = start
    while 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
        if image[y, x] == 255:
            return x, y
        x += direction[0]
        y += direction[1]
    return None

map_file_path = "/home/jetbot/catkin_ws/src/jetbot_pro/maps/mymap.yaml"
with open(map_file_path, 'r') as stream:
    map_metadata = yaml.safe_load(stream)

resolution = map_metadata["resolution"]
origin = map_metadata["origin"]

map_image_path = map_file_path.replace("yaml", "pgm")
image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)

_, binary_map = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY)
edges = cv2.Canny(binary_map, threshold1=50, threshold2=150)

# 각 모서리의 시작점과 방향 설정
corners = {
    "top_left": ((0, 0), (1, 1)),
    "top_right": ((0, image.shape[1] - 1), (1, -1)),
    "bottom_left": ((image.shape[0] - 1, 0), (-1, 1)),
    "bottom_right": ((image.shape[0] - 1, image.shape[1] - 1), (-1, -1))
}

# 각 모서리의 첫 번째 가장자리 픽셀 찾기
corner_coords = {}
for corner, (start, direction) in corners.items():
    pixel = find_first_edge_pixel(edges, start, direction)
    if pixel:
        x_real = (pixel[0] * resolution) + origin[0]
        y_real = (pixel[1] * resolution) + origin[1]
        corner_coords[corner] = (x_real, y_real)

# 결과 출력
for corner, coord in corner_coords.items():
    print "%s corner: x = %.2f, y = %.2f" % (corner, coord[0], coord[1])

