#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
import requests
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool, String
import datetime
import numpy as np
from ultralytics.utils import ASSETS, yaml_load
import cv2
from sensor_msgs.msg import CompressedImage

CLASSES = yaml_load("/home/orin/catkin_ws/src/jetbot_pro/scripts/yolov8/coco128.yaml")["names"]
print("CLASSSES: ", CLASSES)
colors = np.random.uniform(0, 255, size=(len(CLASSES), 3))


def draw_bounding_box(img, class_id, confidence, x, y, x_plus_w, y_plus_h):
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
    color = colors[class_id]
    cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)
    cv2.putText(img, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

def download_photo(url, path):
    response = requests.get(url)
    if response.status_code == 200:
        with open(path, 'wb') as f:
            f.write(response.content)
    else:
        print("Failed to download the photo.")

class Listener():
    def __init__(self):
        self.image = None
        self.br = CvBridge()
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        # rospy.Subscriber('imagetimer', Image, callback)
        # pub = rospy.Publisher('chatter', String, queue_size=10)
        self.general_idx = 0

    def callback(self, msg):
        # self.image = self.br.imgmsg_to_cv2(msg)
        #self.image = self.br.compressed_imgmsg_to_cv2(msg, "bgr8")
        #rospy.loginfo('Receive image shape: %s, time: %s', str(self.image.shape), str(datetime.datetime.now()))

        # get image using wget
        print("wget download start!")
        photo_url = "http://172.26.53.31:8123/jetbot_object_detection.png"
        photo_path = "/home/orin/catkin_ws/src/jetbot_pro/scripts/jetbot_object_detection.png"
        download_photo(photo_url, photo_path)
        self.image = cv2.imread(photo_path)
        print("self image: ", self.image)
        rospy.loginfo('Receive wget image shape: %s, time: %s', str(self.image.shape), str(datetime.datetime.now()))

        # put object detection model here
        # Load the ONNX model
        # onnx_model = '/home/orin/catkin_ws/src/jetbot_pro/scripts/yolov8/yolov8n.onnx'
        onnx_model = '/home/orin/catkin_ws/src/jetbot_pro/scripts/yolov8/yolov8m.onnx'
        model: cv2.dnn.Net = cv2.dnn.readNetFromONNX(onnx_model)

        [height, width, _] = self.image.shape
        # Prepare a square image for inference
        length = max((height, width))
        # Calculate scale factor
        scale = length / 640

        start = datetime.datetime.now() # fps count start
        blob = cv2.dnn.blobFromImage(self.image, scalefactor=1 / 255, size=(640, 640), swapRB=True)
        model.setInput(blob)

        # Perform inference
        outputs = model.forward()

        # Prepare output array
        outputs = np.array([cv2.transpose(outputs[0])])
        rows = outputs.shape[1]

        boxes = []
        scores = []
        class_ids = []

        # Iterate through output to collect bounding boxes, confidence scores, and class IDs
        for i in range(rows):
            classes_scores = outputs[0][i][4:]
            (minScore, maxScore, minClassLoc, (x, maxClassIndex)) = cv2.minMaxLoc(classes_scores)
            if maxScore >= 0.25:
                box = [
                    outputs[0][i][0] - (0.5 * outputs[0][i][2]),
                    outputs[0][i][1] - (0.5 * outputs[0][i][3]),
                    outputs[0][i][2],
                    outputs[0][i][3],
                ]
                boxes.append(box)
                scores.append(maxScore)
                class_ids.append(maxClassIndex)

        # Apply NMS (Non-maximum suppression)
        result_boxes = cv2.dnn.NMSBoxes(boxes, scores, 0.25, 0.45, 0.5)

        detections = []

        # Iterate through NMS results to draw bounding boxes and labels
        success = False
        for i in range(len(result_boxes)):
            index = result_boxes[i]
            box = boxes[index]
            detection = {
                "class_id": class_ids[index],
                "class_name": CLASSES[class_ids[index]],
                "confidence": scores[index],
                "box": box,
                "scale": scale,
            }
            detections.append(detection)
            draw_bounding_box(
                self.image,
                class_ids[index],
                scores[index],
                round(box[0] * scale),
                round(box[1] * scale),
                round((box[0] + box[2]) * scale),
                round((box[1] + box[3]) * scale),
            )

            class_id = class_ids[index]
            # class_name = CLASSES[class_id]
            #if class_id == 24 or class_id == 26 or class_id == 37: # 24: backpack, 26: handbag, 37: surfboard
            #    class_id = 24
            #    print(CLASSES[class_id], " detected and published!general_idx+=1")
            #    # self.pub.publish("True")
            #    self.pub.publish(str(self.general_idx))
            #    self.general_idx += 1
            #    success = True
        #if success == False:
            # self.pub.publish("False")
        #    self.pub.publish(str(self.general_idx))
        #    self.general_idx += 1
        #    print("general_idx += 1, publish False!")
        end = datetime.datetime.now()
        fps = f"FPS: {1 / (end-start).total_seconds():.2f}"
        print("fps: ", fps)
        cv2.putText(self.image, fps, (50, 50), cv2.FONT_HERSHEY_DUPPLEX, 0.5, (0, 0, 255), 0.5)
        cv2.imwrite('/home/orin/catkin_ws/src/jetbot_pro/scripts/orin_received_image/'+str(end)+'_image.png', self.image)

    def listener(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        self.pub.publish(str(self.general_idx))
        while not rospy.is_shutdown():
            # rospy.Subscriber('imagetimer', Image, self.callback)
            rospy.Subscriber('imagetimer', CompressedImage, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    # listener()
    rospy.init_node('listener', anonymous=True)
    my_node = Listener()
    my_node.listener()
