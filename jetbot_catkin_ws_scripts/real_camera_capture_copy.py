#!/usr/bin/env python
from jetcam.csi_camera import CSICamera
import time
import cv2
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool
import sys


class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        # self.csi_image = csi_image
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub = rospy.Publisher('csi_image', Image,queue_size=10)
        self.sub = rospy.Subscriber("cv_image",Image,self.callback)

        # Subscribers

    def callback(self, msg):
        rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)
        rospy.loginfo("I heard %s", str(self.image.shape))


    def start(self):
        rospy.loginfo("Timing images")
        #rospy.spin()
        # i = 0
        # while i < 5:
        #     csi_image = save_image()
        #     print(type(csi_image))
        #     self.pub.publish(self.br.cv2_to_imgmsg(csi_image))
        #     print("publishesd!")
        #     i = i+1
        while not rospy.is_shutdown():
            # if self.image is not None:
            rospy.loginfo('publishing image')
            #br = CvBridge()
            # if self.image is None:
            csi_image = save_image()
            print(type(csi_image))
            self.pub.publish(self.br.cv2_to_imgmsg(csi_image))
            rospy.Subscriber("cv_image",Image,self.callback)
                    
            self.loop_rate.sleep()

def callback_camera(change):
    new_image = change['new']
    # do some processing...

def save_image():
    image = camera.value
    cv2.imwrite('output.jpg', image)
    return image

        
if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    camera = CSICamera(width=224, height=224, capture_width=1080, capture_height=720, capture_fps=30)
    camera.read()
    camera.running = True
    camera.observe(callback_camera, names='value')
    # while not rospy.is_shutdown():
        # image = camera.value

    my_node = Nodo()
    my_node.start()


    
# def callback(msg):
#     rospy.loginfo('Image received...')
#     bridge = CvBridge()
#     image= bridge.imgmsg_to_cv2(msg)
#     # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)


# def main():
#     br = CvBridge()
#     rospy.init_node('image', anonymous = True)
#     # rospy.init_node('listener', anonymous=True)
#     while not rospy.is_shutdown():
#         try: 

#             image_pub = rospy.Publisher("imagetimer", Image, queue_size = 10)
#             image = save_image()
#             print(type(image))
#             image_pub.publish(br.cv2_to_imgmsg(image))
#             rospy.Subscriber('chatter', Image, callback)
#             print("subscribed!")
#             time.sleep(1)
#             # rospy.spin()
#         except KeyboardInterrupt:
#             print("Stopped by User")
#             camera.running = False



# if __name__=="__main__":
#     # while not rospy.is_shutdown():
#     main()

