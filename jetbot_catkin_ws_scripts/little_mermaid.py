import rospy
from std_msgs.msg import Bool, String
rospy.init_node('MultiPoint_navigation')
pub_tts = rospy.Publisher('/speak', String, queue_size = 10)
for _ in range(2):
    pub_tts.publish("I wanna be where the people are I wanna see wanna see em dancin'")
    rospy.sleep(2)