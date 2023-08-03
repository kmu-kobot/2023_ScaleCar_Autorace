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
        self.warper = Warper()
        self.slidewindow = SlideWindow()
        self.bridge = CvBridge()

        # slide window return variable initialization
        self.initialized = False
        self.slide_img = None 
        self.slide_x_location = 0
        self.current_lane_window = ""

        rospy.Timer(rospy.Duration(1.0/30.0), self.timerCallback)
        self.webot_speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1) # motor speed
        self.webot_angle_pub = rospy.Publisher("/commands/servor/position", Float64, queue_size=1) # servo angle

        rospy.Subscriber("usb_cam/image_rect_color", Image, self.laneCallback)
        rospy.Subscriber("sign_id", Int32, self.child_sign_callback)
    
    def timerCallback(self, _event):
        try:
            self.mainAlgorithm()
        except:
            pass
    
    def laneCallback(self, _data):
        #detect lane
        if self.initialized == False:
            cv2.namedWindow("Simulator_Image", cv2.WINDOW_NORMAL) 
            cv2.createTrackbar('low_H', 'Simulator_Image', 50, 255, nothing)
            cv2.createTrackbar('low_S', 'Simulator_Image', 50, 255, nothing)
            cv2.createTrackbar('low_V', 'Simulator_Image', 50, 255, nothing)
            cv2.createTrackbar('high_H', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_S', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_V', 'Simulator_Image', 255, 255, nothing)
            self.initialized = True
        
        cv2_image = self.bridge.imgmsg_to_cv2(_data)
        self.obstacle_img = cv2_image

        cv2.imshow("original", cv2_image) 

        low_H = cv2.getTrackbarPos('low_H', 'Simulator_Image')
        low_S = cv2.getTrackbarPos('low_S', 'Simulator_Image')
        low_V = cv2.getTrackbarPos('low_V', 'Simulator_Image')
        high_H = cv2.getTrackbarPos('high_H', 'Simulator_Image')
        high_S = cv2.getTrackbarPos('high_S', 'Simulator_Image')
        high_V = cv2.getTrackbarPos('high_V', 'Simulator_Image')

        cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV) # BGR to HSV

        lower_lane = np.array([low_H, low_S, low_V]) # 
        upper_lane = np.array([high_H, high_S, high_V])

        lane_image = cv2.inRange(cv2_image, lower_lane, upper_lane)

        cv2.imshow("Lane Image", lane_image)
        self.laneDetection(lane_image)

        cv2.waitKey(1)
    
    def laneDetection(self, lane_image):
        kernel_size = 5
        blur_img = cv2.GaussianBlur(lane_image,(kernel_size, kernel_size), 0)
        warped_img = self.warper.warp(blur_img)
        cv2.imshow("warped_img", warped_img)
        self.slide_img, self.slide_x_location, self.current_lane_window = self.slidewindow.slidewindow(warped_img)
        cv2.imshow("slide_img", self.slide_img)
        # rospy.loginfo("CURRENT LANE WINDOW: {}".format(self.current_lane_window))

    def child_sign_callback(self, _data):
        #updating...
        return 0
    
    def mainAlgorithm(self):
        #defalut driving
        speed_msg = Float64() # speed msg create
        angle_msg = Float64() # angle msg create
        speed_msg.data = 500 # defalut speed
        angle_msg.data = self.slide_x_location - 0.165 # calculate angle error with slingwindow data (!!tuning required!!)
        self.webot_speed_pub.publish(speed_msg) # publish speed
        self.webot_angle_pub.publish(angle_msg) # publish angle
        
def nothing(x):
    pass

def run():

    rospy.init_node("main_class_run")
    control = MainLoop()
    rospy.Timer(rospy.Duration(1.0/30.0), control.timerCallback) 
    rospy.spin()

if __name__ == "__main__":
    run()