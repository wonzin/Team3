#!/usr/bin/env python
from jetcam.csi_camera import CSICamera
import time
import cv2
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool
import sys
import datetime
from gtts import gTTS

camera = CSICamera(width=224, height=224, capture_width=1080, capture_height=720, capture_fps=30)
camera.read()
camera.running = True

def callback(change):
    new_image = change['new']

camera.observe(callback, names='value')
image = camera.value

def save_image():
    image = camera.value
    cv2.imwrite('/home/jetbot/catkin_ws/src/jetbot_pro/scripts/jetbot_picture_test/'+str(datetime.datetime.now())+'.png', image)
    return image
    
def callback_string(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    is_sub_string += 1
    tts = gTTS(text="I heard "+data.data, lang='en-us')
    tts.save('/home/jetbot/catkin_ws/src/jetbot_pro/scripts/demo.mp3')
    p=Popen("play -q /home/jetbot/catkin_ws/src/jetbot_pro/scripts/demo.mp3", stdout=PIPE, shell=True)
    p.wait()
    os.remove('/home/jetbot/catkin_ws/src/jetbot_pro/scripts/demo.mp3')


def main():
    br = CvBridge()
    rospy.init_node('image', anonymous = True)
    while not rospy.is_shutdown():
        try:
            # image_pub = rospy.Publisher("imagetimer", Image, queue_size = 10)
            image_pub = rospy.Publisher("imagetimer", CompressedImage, queue_size=10)
            image = save_image()
            print(type(image))
            image_pub.publish(br.cv2_to_compressed_imgmsg(image))
            rospy.Subscriber('chatter', String, callback_string)
            print("subscribed!")
            # rospy.spin()
        except KeyboardInterrupt:
            print("Stopped by User")
            camera.running = False



if __name__=="__main__":
    # while not rospy.is_shutdown():
    main()

