#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
from time import sleep, time
import statistics

from std_msgs.msg import Int32, String, Float32, Float64
from sensor_msgs.msg import Image
from obstacle_detector.msg import Obstacles

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

        self.current_lane = "RIGHT"
        self.is_safe = True
        self.initialized = False

        # slide window return variable initialization
        self.initialized = False
        self.slide_img = None 
        self.slide_x_location = 0
        self.current_lane_window = ""

        # for child sign
        self.is_child_detected = False
        self.slow_t1 = 0.0
        self.sign_data = 0

        # for rubbercon misson
        self.is_rubbercon_mission = False
        self.rubbercon_angle_error = 0

        # for static obstacle
        self.is_static_mission = False
        self.turn_left_flag = False
        self.turn_right_flag = False

        # for dynamic obstacle
        self.is_dynamic_mission = False

        self.obstacle_img = []

        self.y_list = []
        self.y_list_sort = []
        self.dynamic_obs_cnt = 0
        self.static_cnt = 0

        rospy.Timer(rospy.Duration(1.0/30.0), self.timerCallback)
        self.webot_speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1) # motor speed
        self.webot_angle_pub = rospy.Publisher("/commands/servor/position", Float64, queue_size=1) # servo angle

        rospy.Subscriber("usb_cam/image_rect_color", Image, self.laneCallback)
        rospy.Subscriber("sign_id", Int32, self.child_sign_callback)
        rospy.Subscriber("rubber_cone", Float32, self.rubbercone_callback)

        rospy.Subscriber("lidar_warning", String, self.warning_callback) # lidar 에서 받아온 object 탐지 subscribe (warning / safe)
        rospy.Subscriber("object_condition", Float32, self.object_callback) 


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
        try :
            if _data.data == 3:
                self.child_cnt += 1
                if self.child_cnt >=20 :
                    self.sign_data = _data.data
                    self.is_child_detected = True
                    self.child_cnt = 0
            else :
                self.sign_data = 0
        except :
            pass

    def warning_callback(self, _data):
        if self.is_rubbercon_mission == True :
            self.is_rubbercon_mission = True
        elif _data.data == "safe":
            self.is_safe = True
            self.y_list = []
            if self.is_dynamic_mission == 1 :
                self.dynamic_obs_cnt += 1
                if self.dynamic_obs_cnt >= 50 : # 50
                    self.is_dynamic_mission = 0
                    self.dynamic_obs_cnt = 0
            self.is_static_mission = 0
        elif _data.data == "WARNING":
            self.is_safe = False
            rospy.loginfo("WARNING!")
        else:
            pass

    def object_callback(self, _data):
        if self.is_dynamic_mission != False or len(self.y_list) <= 19:
            self.y_list.append(_data.data)
            if len(self.y_list) >= 21 :
                del self.y_list[0]
        else:
            rospy.logwarn("Unknown warning state!")

    def rubbercone_callback(self, _data):
        self.rubbercon_angle_error = _data.data
        if self.rubbercon_angle_error < 10.0 :
            self.is_rubbercon_mission = True
        else :
            self.is_rubbercon_mission = False


    def mainAlgorithm(self):
        speed_msg = Float64() # speed msg create
        angle_msg = Float64() # angle msg create

        # child protect driving
        if self.is_child_detected == True:
            # child sign detected, waiting sign to disappear.
            if self.sign_data == 3:
                angle_msg.data = 280 - self.slide_x_location # calculate angle error with sliding window data
                speed_msg.data = 1000 # defalut speed

            # sign disappered, drive slow
            elif self.sign_data == 0 :      
                if self.slow_flag == 0:
                    self.slow_t1 = rospy.get_time() # start time
                    self.is_child_detected = True
                t2 = rospy.get_time() # time counter

                # drive slow during 15seconds
                while t2-self.slow_t1 <= 15 :
                    angle_msg = 280 - self.slide_x_location # calculate angle error with sliding window data
                    speed_msg = 500 #slow speed
                    self.webot_speed_pub.publish(speed_msg) # publish speed
                    self.webot_angle_pub.publish(angle_msg) # publish angle
                    t2 = rospy.get_time()
                self.slow_down_flag = 0
                self.slow_flag = 0
        
        # rubbercon mission
        elif self.is_rubbercon_mission == True:
            self.rubbercon_drive()
            self.current_lane = "RIGHT"
        
        # obstacle mission
        elif self.is_safe == False:
            self.y_list_sort = sorted(self.y_list, key = lambda x:x)
            if len(self.y_list) <= 19 :
                self.stop()
            elif abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])) >= 0.17 or self.y_list_sort[10] < -0.15:
                self.is_dynamic_mission = True
                self.is_static_mission = False
                # self.y_list.clear
                rospy.loginfo("dynamic")
                # rospy.loginfo(self.y_list_sort)
                rospy.loginfo(abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])))
            else:
                self.static_cnt += 1
                if self.static_cnt >= 10 :
                    self.is_static_mission = True
                    self.is_dynamic_mission = False
                    self.static_cnt = 0
                rospy.loginfo("static")
                # rospy.loginfo(self.y_list_sort)
                rospy.loginfo(abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])))

            #dynamic
            if self.is_dynamic_mission == True and self.is_safe == False :
                rospy.logwarn("DYNAMIC OBSTACLE")
                self.stop()



            # static obstacle
            elif self.is_static_mission == 1:
                rospy.loginfo("STATIC OBSTACLE")
                # if the car is driving depending on "right" window
                if self.current_lane == "RIGHT":
                    if self.turn_left_flag == 0:
                        self.turn_left_t1 = rospy.get_time()
                        self.turn_left_flag = 1
                    t2 = rospy.get_time()
                    
                    # go to left line
                    while t2-self.turn_left_t1 <= 1.0:
                        self.change_line_left()
                        t2 = rospy.get_time()
                    # array car to fit line
                    while t2- self.turn_left_t1 <= 1.25 :
                        self.change_line_right()
                        t2 = rospy.get_time()
                    self.current_lane = "LEFT"
                    self.is_static_mission = 0
                    self.turn_left_flag = 0

                # if the car is driving depending on "left" window
                elif self.current_lane == "LEFT":
                    if self.turn_right_flag == 0:
                        self.turn_right_t1 = rospy.get_time()
                        self.turn_right_flag = 1
                    t2 = rospy.get_time()
                    
                    # go to right line 
                    while t2-self.turn_right_t1 <= 1.0:
                        self.change_line_right()
                        t2 = rospy.get_time()
                    # array car to fit line
                    while t2-self.turn_right_t1 <= 1.25 :
                        self.change_line_left()
                        t2 = rospy.get_time()
                    self.current_lane = "RIGHT"
                    self.is_static_mission = 0
                    self.turn_right_flag = 0

        # defalut driving
        else:
            speed_msg.data = 1000 # defalut speed
            angle_msg.data = self.slide_x_location - 0.165 # calculate angle error with slingwindow data (!!tuning required!!)
            self.webot_speed_pub.publish(speed_msg) # publish speed
            self.webot_angle_pub.publish(angle_msg) # publish angle

    def rubbercon_drive(self) :
        rospy.loginfo("rubbercon_drive!!!!!!!!!!!!!!!!!")
        speed_msg = Float64() # speed msg create
        angle_msg = Float64() # angle msg create
        
        speed_msg.data = 500 # rubbercon speed
        angle_msg.data = self.rubbercon_angle_error * -1.0

        self.webot_speed_pub.publish(speed_msg) # publish speed
        self.webot_angle_pub.publish(angle_msg) # publish angle
    
    def change_line_left(self) :
        speed_msg = Float64() # speed msg create
        angle_msg = Float64() # angle msg create
        speed_msg.data = 300.0
        angle_msg.data = 0.3
        self.webot_speed_pub.publish(speed_msg)
        self.webot_angle_pub.publish(angle_msg)

    def change_line_right(self) :
        speed_msg = Float64() # speed msg create
        angle_msg = Float64() # angle msg create
        speed_msg.data = 300.0
        angle_msg.data = -0.3
        self.webot_speed_pub.publish(speed_msg)
        self.webot_angle_pub.publish(angle_msg)

    def stop(self):
        speed_msg = Float64() # speed msg create
        angle_msg = Float64() # angle msg create
        speed_msg.data = 0.0
        angle_msg.data = 0.0
        self.webot_speed_pub.publish(speed_msg)
        self.webot_angle_pub.publish(angle_msg)
        

def nothing(x):
    pass

def run():

    rospy.init_node("main_class_run")
    control = MainLoop()
    rospy.Timer(rospy.Duration(1.0/30.0), control.timerCallback) 
    rospy.spin()

if __name__ == "__main__":
    run()