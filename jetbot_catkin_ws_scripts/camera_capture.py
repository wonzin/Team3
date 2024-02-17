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
camera = CSICamera(width=224, height=224, capture_width=1080, capture_height=720, capture_fps=30)
camera.read()
camera.running = True
from gtts import gTTS

def callback(change):
    new_image = change['new']
    # do some processing...

br = CvBridge()
camera.observe(callback, names='value')
image = camera.value
image_pub = rospy.Publisher("imagetimer", CompressedImage, queue_size=10)
sub_int = 0

def save_image():
    image = camera.value
    cv2.imwrite('/home/jetbot/catkin_ws/src/jetbot_pro/scripts/object_detection_jetbot/jetbot_object_detection.png', image)
    # image_pub shouldn't be here
    return image
    
def callback_string(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    tts = gTTS(text="I heard "+data.data, lang='en-us')
    tts.save('/home/jetbot/catkin_ws/src/jetbot_pro/scripts/demo.mp3')
    p=Popen("play -q /home/jetbot/catkin_ws/src/jetbot_pro/scripts/demo.mp3", stdout=PIPE, shell=True)
    p.wait()
    os.remove('/home/jetbot/catkin_ws/src/jetbot_pro/scripts/demo.mp3')
    print("callback_string: "+data.data+", sub_int: "+str(sub_int))
    if int(data.data) != sub_int:
        image_pub.publish(br.cv2_to_compressed_imgmsg(image))
        sub_int = int(data.data)

def main():
    # br = CvBridge()
    rospy.init_node('image', anonymous = True)
    # rospy.init_node('listener', anonymous=True)
    while not rospy.is_shutdown():
        try: 
            # image_pub = rospy.Publisher("imagetimer", Image, queue_size = 10)
            # image_pub = rospy.Publisher("imagetimer", CompressedImage, queue_size=10)
            image = save_image()
            print(type(image))
            image_pub.publish(br.cv2_to_compressed_imgmsg(image))
            rospy.Subscriber('chatter', String, callback_string)
            print("subscribed!")
            rospy.sleep(1)
            # rospy.spin()
        except KeyboardInterrupt:
            print("Stopped by User")
            camera.running = False



if __name__=="__main__":
    # while not rospy.is_shutdown():
    main()

