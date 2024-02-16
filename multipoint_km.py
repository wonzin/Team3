#!/usr/bin/env python
# encoding: utf-8
import rospy
from std_msgs.msg import Bool
from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalID
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Twist

import math


class Multipoint_navigation:
    def __init__(self):
        # Initialize node
        rospy.init_node('MultiPoint_navigation')
        rospy.on_shutdown(self.cancel)
        
        # Target point marker array
        self.markerArray = MarkerArray()
        # point count
        self.count = 0
        # point index
        self.index = 0
        # Allow another attempt to go to the target point that has not been reached
        self.try_again = 1
        # 
        self.target_arrival = 0
        self.qr_iden = 0
        self.cur_x = 0
        self.cur_y = 0
        self.angle = 20
        
        # Used to publish target point markers
        self.pub_mark = rospy.Publisher('/path_point', MarkerArray, queue_size=100)
        # Subscribe to mark the pressed position in rviz
        self.sub_click = rospy.Subscriber('/clicked_point', PointStamped, self.click_callback)
        # Publish target point
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        # cancel target point
        self.pub_cancelgoal = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        # Subscribe to the status of reaching the target point
        self.sub_goal_result = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.goal_result_callback)
        # Subscribe to the initial pose topic
        self.sub_initialpose = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialpose_callback)
        # Post initial pose
        self.pub_rtabinitPose = rospy.Publisher("/rtabmap/initialpose", PoseWithCovarianceStamped, queue_size=10)
        # find qr code
        self.founded = rospy.Subscriber('/qr_found', Bool, self.qr_search_callback)
        self.change_motor = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            # Publish markerArray
            self.pub_mark.publish(self.markerArray)
            self.rate.sleep()
        
    def cancel(self):
        self.pub_cancelgoal.publish(GoalID())
        self.pub_mark.unregister()
        self.pub_goal.unregister()
        self.pub_cancelgoal.unregister()
        self.pub_rtabinitPose.unregister()
        self.sub_click.unregister()
        self.sub_goal_result.unregister()
        self.sub_initialpose.unregister()

    def initialpose_callback(self, msg):
        if not isinstance(msg, PoseWithCovarianceStamped): return
        
        # Modify this function to reset relative position of jetbot (2d pose estimation on rviz)
        # need to save this when jetbot arrived at the goal and use this value after the jetbot identifies QR code
        
        # Clear marker
        self.markerArray = MarkerArray()
        marker = Marker()
        marker.action = marker.DELETEALL
        self.markerArray.markers.append(marker)
        self.pub_mark.publish(self.markerArray)
        self.markerArray = MarkerArray()
        self.count = 0
        self.index = 0
        self.try_again = 1
        self.pub_cancelgoal.publish(GoalID())
        self.pub_rtabinitPose.publish(msg)
        print(msg)

    def click_callback(self, msg):
        print('Add a new target point ' + str(self.count) + '.')
        
        # Modify a position XYZ
        if self.count==0:
            msg.point.x = 2.7
            msg.point.y = -4.2
        elif self.count==1:
            msg.point.x = 0.9
            msg.point.y = 0.9
        
        # Create a marker object
        marker = Marker()
        marker.header.frame_id = 'map'
        # Character format
        marker.type = marker.TEXT_VIEW_FACING
        # marker model
        marker.action = marker.ADD
        # the size of the marker
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.6
        # marker ColorRGBA
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1
        # marker position XYZ
        marker.pose.position.x = msg.point.x
        marker.pose.position.y = msg.point.y
        marker.pose.position.z = msg.point.z
        # marker text
        marker.text = str(self.count)
        self.markerArray.markers.append(marker)
        # Set the id of markers
        id = 0
        for m in self.markerArray.markers:
            m.id = id
            id += 1

        # Publish target point
        if self.count == 0:
            self.PubTargetPoint(msg.point.x, msg.point.y)
            self.cur_x = msg.point.x
            self.cur_y = msg.point.y
            self.index += 1
        self.count += 1
        print(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)

    def PubTargetPoint(self, x, y, z = 0, w = 1):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        # The location of the target point
        pose.pose.position.x = x
        pose.pose.position.y = y
        # The posture of the target point. z=sin(angle/2) w=cos(angle/2)
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        self.pub_goal.publish(pose)

    def goal_result_callback(self, msg):
        
        if self.count == 0: return
        print ("Get the status of reaching the target point!!!")
        # Reach the target point
        if msg.status.status == 3 or self.target_arrival == 1:
            if self.qr_iden == 0:
                self.target_arrival = 1
                print("no qr but target arrived")
        # Did not reach the target point
        else :
            rospy.logwarn('Can not reach the target point ' + str(self.index - 1) + '.')
            # Try again to reach the unreached target point
            if self.try_again == 1:
                rospy.logwarn('trying reach the target point ' + str(self.index - 1) + ' again!')
                x = self.markerArray.markers[self.index - 1].pose.position.x
                y = self.markerArray.markers[self.index - 1].pose.position.y
                self.PubTargetPoint(x, y)
                # It is not allowed to try again to reach the unreached target point
                self.try_again = 0
                self.qr_iden = 0
            # Continue to the next target point
            elif self.index < len(self.markerArray.markers):
                rospy.logwarn('try reach the target point ' + str(self.index - 1) + ' failed! reach next point.')
                # If this round of cruise has been completed, the setting starts from the beginning
                if self.index == self.count: self.index = 0
                x = self.markerArray.markers[self.index].pose.position.x
                y = self.markerArray.markers[self.index].pose.position.y
                self.PubTargetPoint(x, y)
                # Cruise to the next point
                self.index += 1
                # Allow another attempt to reach the unreached target point
                self.try_again = 1
                self.qr_iden = 0
                
                
    def qr_search_callback(self, msg):
        # if qr code is detected at current position/frame, do tts
        # if qr code is not detected, rotate/move jetbot
        # qr code detection: orin --> get value from orin!
        
        # msg == 0: qr not detected
        # msg == 1: qr detected
        if msg.data:
            # qr detected
            if self.target_arrival == 1:
                twist = Twist()
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0
                self.change_motor.publish(twist)
                
                self.try_again = 1
                #  This round of cruise is completed, restart cruise
                if self.index == self.count:
                    print ('Reach the target point ' + str(self.index - 1) + '.')
                    self.index = 0
                    x = self.markerArray.markers[self.index].pose.position.x
                    y = self.markerArray.markers[self.index].pose.position.y
                    self.cur_x = x
                    self.cur_y = y
                    self.PubTargetPoint(x, y)
                    # Cruise to the next point
                    self.index += 1
                    self.target_arrival = 0
                    self.qr_iden = 0
                # Cruise the remaining points of the round
                elif self.index < self.count:
                    print ('Reach the target point ' + str(self.index - 1) + '.')
                    x = self.markerArray.markers[self.index].pose.position.x
                    y = self.markerArray.markers[self.index].pose.position.y
                    self.cur_x = x
                    self.cur_y = y
                    self.PubTargetPoint(x, y)
                    # Cruise to the next point
                    self.index += 1
                    self.target_arrival = 0
                    self.qr_iden = 0
            else:
                self.qr_iden = 0
        else:
            # qr not detected
            if self.target_arrival == 1:
                # rotate, move jetbot
                twist = Twist()
                twist.angular.x = 0 #math.radians(self.angle)
                twist.angular.y = 0
                twist.angular.z = math.radians(self.angle)
                self.change_motor.publish(twist)
    
if __name__ == '__main__':
    Multipoint_navigation()

