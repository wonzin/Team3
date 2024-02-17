#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy 
from std_msgs.msg import String
from gtts import gTTS
import os
from subprocess import Popen, PIPE
import requests.packages.urllib3
requests.packages.urllib3.disable_warnings()

FilePath = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/data" #../data

def callback(data):
    rospy.loginfo('I heard %s', data.data)
    try:
        tts = gTTS(text=data.data, lang='en-us')
        # tts = gTTS(text='scotty detected', lang='en-us')
        # tts.save(FilePath + "/demo.mp3")
        tts.save('/home/jetbot/catkin_ws/src/jetbot_pro/scripts/demo.mp3')
        # p=Popen("play -q "+ FilePath + "/demo.mp3", stdout=PIPE, shell=True)
        p=Popen("play -q /home/jetbot/catkin_ws/src/jetbot_pro/scripts/demo.mp3", stdout=PIPE, shell=True)
        p.wait()
        # os.remove(FilePath + '/demo.mp3')
        os.remove('/home/jetbot/catkin_ws/src/jetbot_pro/scripts/demo.mp3')
    except Exception as e:
        print("receive msg,but parse exception:", e)

if __name__ == '__main__':
    rospy.init_node('jetbot_tts_node', anonymous=True)
    rospy.Subscriber('/speak_object', String, callback, queue_size=1)
    rospy.spin()
