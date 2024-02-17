#!/usr/bin/env python
# encoding: utf-8
import rospy
import datetime
import os
import cv2
from sensor_msgs.msg import CompressedImage
from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
import tf
import random
import math
import numpy as np
# from sensor_msgs.msg import Image
from jetcam.csi_camera import CSICamera
from cv_bridge import CvBridge
from std_msgs.msg import String
from threading import Event

# 실제 좌표로 변환하는 로직
# ...

class Multipoint_navigation:
    def __init__(self):
        # Initialize node
        rospy.init_node('MultiPoint_navigation')
        rospy.on_shutdown(self.cancel)       

        # # Transform listener 추가
        # smove_baseelf.tf_listener = tf.TransformListener()
        # Target point marker array
        self.markerArray = MarkerArray()
        # point count
        self.count = 0
        self.picture_cnt = 0
        # point index
        self.index = 0
        # Allow another attempt to go to the target point that has not been reached
        self.try_again = 1
        
        # Used to publish target point markers
        self.pub_mark = rospy.Publisher('/path_point', MarkerArray, queue_size=100)
        # Subscribe to mark the pressed position in rviz
        # self.sub_click = rospy.Subscriber('/clicked_point', PointStamped, self.click_callback) # 마우스 클릭으로 입력 받을 시 필요. 
        # Publish target point
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        # cancel target point
        self.pub_cancelgoal = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        # Subscribe to the status of reaching the target point
        self.sub_goal_result = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.goal_result_callback)
        # Subscribe to the initial pose topic
        self.sub_initialpose = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialpose_callback)
        
        # 이미지 보내는 subcriber
        # self.chatter_subscriber = rospy.Subscriber('chatter', String, self.callback_string_rotate, queue_size=1)
        # self.chatter_subscriber.unregister()  # 초기에는 구독을 비활성화
        
        # Post initial pose
        self.pub_rtabinitPose = rospy.Publisher("/rtabmap/initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.angle = 20
        self.change_motor = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        
        self.pub_cancelgoal = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        # self.image_pub = rospy.Publisher("csi_image", Image, queue_size=10)
        self.camera = CSICamera(width=224, height=224, capture_width=1080, capture_height=720, capture_fps=30)
        
        self.camera.read()
        self.camera.running = True
        
        self.camera.observe(self.callback_camera, names = 'value')
        self.br = CvBridge()
        self.image_pub = rospy.Publisher("imagetimer", CompressedImage, queue_size=10)
        self.if_detected = False
        self.image_processed = False

        rate = rospy.Rate(10) # 10hz
        # self.set_target_point(1.0, 1.0)
        # self.rotate_robot()
        self.boundary = {
            "top_left": (-0.943, -0.0713),
            "top_right": (-0.05, -0.115),
            "bottom_left": (-0.0486, -0.981),
            "bottom_right": (-0.894, -0.997)
        }
        self.center_x = (self.boundary["top_left"][0] + self.boundary["bottom_right"][0]) / 2.0
        self.center_y = (self.boundary["top_left"][1] + self.boundary["bottom_right"][1]) / 2.0
       
        cnt = 0
        self.rotation_count = 0
        detected = False 
        
        while not rospy.is_shutdown():
            self.picture_cnt = 0
            user_input = raw_input("Enter random coordinates to start (format: x y): ")
            x, y = map(float, user_input.split())
            print("Type of x:", type(x))
            print("Type of y:", type(y))
            print(x, y)
            self.set_target_point(x, y)
            rospy.wait_for_message('/move_base/result', MoveBaseActionResult)
            detected = self.take_photo_sync()
            print('Detect result:', detected)          
            rospy.sleep(3)

    def save_image(self):
        image = self.camera.value
        cv2.imwrite('/home/jetbot/catkin_ws/src/jetbot_pro/scripts/jetbot_picture_test/'+str(datetime.datetime.now())+'_picture_test.png', image)
        return image

    def callback_camera(self, change):
        new_image = change['new']

    def random_move_within_boundary(self, move_count=5): # 횟수 제한 설정 가능
        # 경계 내에서 무작위 좌표 생성
        for _ in range(move_count):
            x_min = min(self.boundary["top_left"][0], self.boundary["bottom_left"][0])
            x_max = max(self.boundary["top_right"][0], self.boundary["bottom_right"][0])
            y_min = min(self.boundary["bottom_left"][1], self.boundary["bottom_right"][1])
            y_max = max(self.boundary["top_left"][1], self.boundary["top_right"][1])

            random_x = random.uniform(x_min, x_max)
            random_y = random.uniform(y_min, y_max)

            # 생성된 무작위 좌표로 목표점 설정
            self.set_target_point(random_x, random_y)
            rospy.wait_for_message('/move_base/result', MoveBaseActionResult)
            
            print("Random target point: x = {:.2f}, y = {:.2f}".format(random_x, random_y))
            self.rotate_robot()   ## 회전하는 코드 
            # self.take_photo()
            rospy.sleep(1)  # 잠시 대기

        print("Completed random movements within boundary")

    def callback_string(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        if data.data == 'True':
            self.if_detected = True

            
            
    def detection_callback(self, msg):
        # 객체 감지 결과를 처리하는 콜백 함수
        if msg.data == "yes":
            self.detected = True
        else:
            self.detected = False
        self.detection_event.set()            
  

    def callback_string_rotate(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        self.picture_cnt+=1
        print(self.picture_cnt)
        # self.picture_cnt가 1보다 크면 메소드 실행을 멈춤
        if self.picture_cnt > 1:
            print("Already processed a picture, exiting method.")
            self.image_processed = True
            self.chatter_subscriber.unregister()
            self.chatter_subscriber = None
            return
        if data.data == 'True' and self.picture_cnt==1:
            self.if_detected = True
            print('We got True')
#             self.chatter_subscriber.unregister()
        if data.data == 'False'and self.picture_cnt==1:
            twist = Twist()
            print('rotating in parts...')
            quarter_rotation = math.radians(30)
            angular_speed = 2.0
            twist.angular.z = angular_speed
            rotation_time = quarter_rotation / angular_speed
            self.change_motor.publish(twist)
            rospy.sleep(rotation_time)  # 계산된 시간 동안 회전          
            twist.angular.z = 0
            self.change_motor.publish(twist)
#             self.chatter_subscriber.unregister()
            

    def take_photo_sync(self):
        # # 카메라를 중지하고 이미지를 읽은 후 다시 실행
        self.image_processed = False
        # self.camera.running = False
        # image = self.camera.read()
        # cv2.imwrite(str(datetime.datetime.now())+'_output.png', image)
        image = self.camera.value

        # Jimin add code below
        print("image: ", type(image), image.shape)
        self.image_pub.publish(self.br.cv2_to_compressed_imgmsg(image))
        image = self.save_image()

        self.chatter_subscriber = rospy.Subscriber('chatter', String, self.callback_string_rotate, queue_size=1)
        # rospy.Subscriber('chatter', String, self.callback_string_rotate)
        # while not self.image_processed:
        #    rospy.sleep(0.1)
        print("subscribed!")        

        # # 카메라 다시 실행
        # self.camera.running = True
        if self.if_detected == True: # If object detection result is True
            self.camera.running = False
#           
            return True
        else: # If object detection result is False
            
            self.camera.running = True
           
            return False          
            
            
            
            
            
            
            
    def take_photo(self):
        # # 카메라를 중지하고 이미지를 읽은 후 다시 실행
        self.camera.running = False
        image = self.camera.read()
        cv2.imwrite(str(datetime.datetime.now()) + '_output.png', image)
        # image = self.camera.value

        # Jimin add code below
        print("image: ", type(image), image.shape)
        self.image_pub.publish(self.br.cv2_to_compressed_imgmsg(image))
        rospy.Subscriber('chatter', String, self.callback_string)
        print("subscribed!")


        # # 현재 시간을 기반으로 한 고유한 파일명 생성
        # timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        # filename = "output_{}.jpg".format(timestamp)

        # # 저장 경로 설정
        # save_path = "/home/jetbot/catkin_ws/src/jetbot_pro/data"
        # full_path = os.path.join(save_path, filename)

        # # 이미지 저장
        # cv2.imwrite(full_path, image)
        # rospy.loginfo("Photo saved as {}".format(full_path))

        # soojin write code below
        # image_msg = self.br.cv2_to_imgmsg(image, "bgr8")
        # self.image_pub.publish(image_msg)
        # rospy.loginfo("Photo published")
        # cv2.imwrite('output.jpg', image)
        

        # # 카메라 다시 실행
        # self.camera.running = True
        if self.if_detected == True: # If object detection result is True
            self.camera.running = False
            return True
        else: # If object detection result is False
            self.camera_running = True
            return False

    def object_search_callback(self):
        # 객체 감지 여부를 확인하는 로직 필요
        object_detected = self.check_object_detection()

        if object_detected:
            # 객체 감지 시 수행할 작업
            rospy.loginfo("Object detected at ({}, {})".format(self.cur_x, self.cur_y))
            # 여기에 객체 감지 시 수행할 작업 추가
            return True
        else:
            # 객체 미감지 시 로봇 회전
            self.rotate_robot()
            return False

    def rotate_robot(self):
        # 로봇이 90도씩 4번에 나눠서 회전하는 로직
        twist = Twist()
        print('rotating in parts...')

        # 90도 회전을 라디안으로 변환
        quarter_rotation = math.radians(30)

        # 회전하도록 각속도 설정
        angular_speed = 2.0  # 각속도 설정

        twist.angular.z = angular_speed

        # 각속도에 따른 총 회전 시간 계산 (90도 회전)
        rotation_time = quarter_rotation / angular_speed

        # 움직임 카운팅
        move_count = 0

        # 4번에 나누어 회전
        for _ in range(12):
            move_count += 1
            print("Moving, count:", move_count)
            self.change_motor.publish(twist)
            rospy.sleep(rotation_time)  # 계산된 시간 동안 회전          
            twist.angular.z = 0
            self.change_motor.publish(twist)
            # self.take_photo()
            rospy.sleep(3)  # 다음 회전 전에 1초 대기
            detected = self.take_photo_sync()
            rospy.sleep(3)

            
            if detected:
                print("Object detected, stopping rotation.")
                break  # 객체가 감지되면 회전 중단
            else:
                print("No object detected, continuing rotation.")



            # 다음 회전을 위해 각속도 다시 설정
            twist.angular.z = angular_speed

        # 마지막 회전 후 정지
        twist.angular.z = 0
        self.change_motor.publish(twist)
    




    def check_object_detection(self):
        # 여기에 객체 감지 여부를 확인하는 로직 추가
        # 예: self.founded 혹은 외부 센서/알고리즘으로부터 객체 감지 여부를 받아옴
        return False  # 임시로 False 반환

    # def rotate_robot(self):
    #     # 로봇 회전 로직
    #     twist = Twist()
    #     twist.angular.z = math.radians(self.angle)
    #     self.change_motor.publish(twist)
    #     rospy.sleep(1)  # 잠시 대기

    def cancel(self):
        self.pub_cancelgoal.publish(GoalID())
        self.pub_mark.unregister()
        self.pub_goal.unregister()
        self.pub_cancelgoal.unregister()
        self.pub_rtabinitPose.unregister()
        # self.sub_click.unregister()
        self.sub_goal_result.unregister()
        self.sub_initialpose.unregister()

    def initialpose_callback(self, msg):
        if not isinstance(msg, PoseWithCovarianceStamped): return
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

    def set_target_point(self, x, y):
        """입력 받은 좌표로 목표점 설정"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        self.pub_goal.publish(pose)



        
    def start_full_map_exploration(self):
            # 맵의 크기와 탐색 그리드 설정
            map_width = 3  # 예시 값
            map_height = 3  # 예시 값
            grid_size = 1  # 예시 값

            # 모든 그리드 포인트를 순회하며 탐색
            for x in range(0, map_width, grid_size):
                for y in range(0, map_height, grid_size):
                    self.set_target_point(x, y)
                    # 현재 위치에 도달할 때까지 대기
                    rospy.wait_for_message('/move_base/result', MoveBaseActionResult)
                    # 필요한 경우 현재 위치 저장 또는 처리
            print('finished')

    def PubTargetPoint(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        # The location of the target point
        pose.pose.position.x = x
        pose.pose.position.y = y
        # The posture of the target point. z=sin(angle/2) w=cos(angle/2)
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        self.pub_goal.publish(pose)



    def goal_result_callback(self, msg):

        if self.count == 0: return
        print ("Get the status of reaching the target point!!!")
        # Reach the target point
        if msg.status.status == 3:
            self.try_again = 1
            self.take_photo()
            #  This round of cruise is completed, restart cruise
            if self.index == self.count:
                print ('Reach the target point ' + str(self.index - 1) + '.')
                self.index = 0
                x = self.markerArray.markers[self.index].pose.position.x
                y = self.markerArray.markers[self.index].pose.position.y
                self.PubTargetPoint(x, y)
                # Cruise to the next point
                self.index += 1
            # Cruise the remaining points of the round
            elif self.index < self.count:
                print ('Reach the target point ' + str(self.index - 1) + '.')
                x = self.markerArray.markers[self.index].pose.position.x
                y = self.markerArray.markers[self.index].pose.position.y
                self.PubTargetPoint(x, y)
                # Cruise to the next point
                self.index += 1
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

if __name__ == '__main__':
    Multipoint_navigation()
