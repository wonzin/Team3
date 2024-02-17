import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import datetime

class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub = rospy.Publisher('cv_image', Image,queue_size=10)

        # Subscribers
        rospy.Subscriber("csi_image",Image,self.callback)

    def callback(self, msg):
        rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)
        rospy.loginfo(rospy.get_caller_id() + 'I got image, shape: %s %s', str(self.image.shape), str(datetime.datetime.now()))

    def start(self):
        rospy.loginfo("Timing images")
        # rospy.spin()
        while not rospy.is_shutdown():
            # rospy.loginfo('publishing image')
            #br = CvBridge()
            print("Inside shut down")
            # Subscribers
            # rospy.Subscriber("csi_image",Image,self.callback)
            if self.image is not None:
                rospy.loginfo('publishing image %s', str(self.image.shape))
                self.pub.publish(self.br.cv2_to_imgmsg(self.image))
            self.loop_rate.sleep()
            
if __name__ == '__main__':
    rospy.init_node("imagetimer121", anonymous=True)
    my_node = Nodo()
    my_node.start()
