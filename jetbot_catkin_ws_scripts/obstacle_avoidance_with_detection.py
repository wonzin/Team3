# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    # 라이다 데이터 중 가장 가까운 포인트 찾기
    closest_range = min(msg.ranges)
    if closest_range < 1.0:  # 1미터 이내의 장애물 감지
        rospy.loginfo("Closest obstacle at: {:.2f} meters".format(closest_range))

def listener():
    rospy.init_node('laser_scan_listener')
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()