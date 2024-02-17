#%%
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool, String
import datetime
import numpy as np
# from ultralytics.utils import ASSETS, yaml_load
import cv2
from sensor_msgs.msg import CompressedImage
import pathlib
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
from PIL import Image

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
    for filename in filenames:
        # Read image
        print(filename)
        img = mpimg.imread(filename)
        # img = "/home/jetbot/catkin_ws/src/jetbot_pro/scripts/calibrate_data/data2024-02-06 10:42:03.442625_calibration.png"
        # img = np.asarray(Image.open(filename))
        plt.imshow(img)
        img = cv2.imread(str(filename))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find chess board corners
        ret, corners = cv2.findChessboardCorners(gray, patternSize=(sh1, sh2))

        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
        else:
            print("Image %s trashed" % (str(filename)))
    print(len(objpoints), len(imgpoints))
    # Calibrate camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    # Save calibration data
    calibrationPath = imageLocation.joinpath("intrinsicCalibration.npz")
    np.savez(str(calibrationPath), mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

if __name__ == "__main__":
    imgpath = "/home/jetbot/catkin_ws/src/jetbot_pro/scripts/calibrate_data"
    save_calibration_data(imgpath)

# %%
