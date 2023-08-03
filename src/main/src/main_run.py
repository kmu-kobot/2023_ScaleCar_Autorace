#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
from time import sleep, time

from std_msgs.msg import Int32, String, Float32, Float64
from sensor_msgs.msg import Image 

from cv_bridge import CvBridge
import cv2
import numpy as np

from warper import Warper
from slidewindow import SlideWindow

class MainLoop:
    
    def __init__(self):
        # rospy.init_node("first_test_node")
        # self.warper = Warper()
        # self.slidewindow = SlideWindow()
        # self.bridge = CvBridge()

        # slide window return variable initialization
        # self.slide_img = None 
        # self.slide_x_location = 0
        # self.current_lane_window = ""

        rospy.Timer(rospy.Duration(1.0/30.0), self.timerCallback)
        self.webot_ctrl_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1) # node 역할 정하기

        # rospy.Subscriber("usb_cam/image_rect_color", Image, self.laneCallback)

    def timerCallback(self, _event):
        try:
            self.mainAlgorithm()
        except:
            pass
    
    # def laneCallback(self):
        # detect lane
        # return 0
    
    def mainAlgorithm(self):
        #defalut driving
        speed_msg = Float64()
        speed_msg.data = 1000
        self.webot_ctrl_pub.publish(speed_msg)
        
    
def run():

    rospy.init_node("main_class_run")
    control = MainLoop()
    rospy.Timer(rospy.Duration(1.0/30.0), control.timerCallback) 
    rospy.spin()

if __name__ == "__main__":
    run()