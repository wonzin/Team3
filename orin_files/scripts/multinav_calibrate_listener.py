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
#%%
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool, String
import datetime
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
import pathlib
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import matplotlib
#%%
def save_calibration_data(imgpath):
    imageLocation = pathlib.Path(imgpath)
    sh1 = 6
    sh2 = 8

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Make object point arrays, used by camera calibration function
    objp = np.zeros((sh1 * sh2, 3), np.float32)
    objp[:, :2] = np.mgrid[0:sh1, 0:sh2].T.reshape(-1, 2)

    # Initialize list
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    # Find all images in path
    filenames = imageLocation.glob("*.png")
    img_folders = []
    for filename in filenames:
        # Read image
        # print(filenames)
        img = mpimg.imread(filename)
        # plt.imshow(img)
        img = cv2.imread(str(filename))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find chess board corners
        ret, corners = cv2.findChessboardCorners(gray, patternSize=(sh1, sh2))
        # print("ret: ", ret)
        # print("corners: ", corners)

        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
    
            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (sh1, sh2), corners, ret)
            plt.imshow(img)
            img_folders.append(filename)
            
        else:
            print("Image %s trashed" % (str(filename)))
    print(len(objpoints), len(imgpoints))
    # Calibrate camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    # Save calibration data
    calibrationPath = imageLocation.joinpath("intrinsicCalibration.npz")
    # np.savez(str(calibrationPath), mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
    np.savez(str(calibrationPath), objpoints = objpoints, imgpoints = imgpoints)
    return img_folders, calibrationPath

def cal_undistort(img, objpoints, imgpoints):
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img.shape[1:], None, None)
    undist = cv2.undistort(img, mtx, dist, None, mtx)
    return undist

def plot_chesscorner(img):
    nx = 6
    ny = 8
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)
    # If found, draw corners
    if ret == True:
        # Draw and display the corners
        chess_img = cv2.drawChessboardCorners(img, (nx, ny), corners, ret)
        # plt.imshow(chess_img)
    # plt.clf()


#%%
class Listener():
    def __init__(self):
        self.image = None
        self.br = CvBridge()
        self.pub = rospy.Publisher('imagesave', String, queue_size=10)
        npz_path = "/home/orin/catkin_ws/src/jetbot_pro/scripts/calibrate_data/intrinsicCalibration.npz"
        dist_arr = np.load(npz_path)
        self.objpoints = dist_arr["objpoints"]
        self.imgpoints = dist_arr["imgpoints"]

    def callback(self, msg):
        self.image = self.br.compressed_imgmsg_to_cv2(msg, "bgr8")
        rospy.loginfo(rospy.get_caller_id() + 'Receive image shape: %s, time: %s', str(self.image.shape), str(datetime.datetime.now()))
        undistorted = cal_undistort(self.image, self.objpoints, self.imgpoints)
        print("rectifying starting")
        self.pub.publish(self.br.cv2_to_compressed_imgmsg(undistorted))

    def listener(self):
        while not rospy.is_shutdown():
            rospy.Subscriber('calibration', CompressedImage, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    my_node = Listener()
    my_node.listener()
