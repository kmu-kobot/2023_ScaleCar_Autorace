#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
import os
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
from evaluateColor import Evaluator

class MainLoop:
    
    def __init__(self):
        self.warper = Warper()
        self.slidewindow = SlideWindow()
        self.bridge = CvBridge()
        self.evalutator = Evaluator()

        self.current_lane = "LEFT"
        self.is_safe = True
        self.initialized = False

        self.lane_msg = Float64() # currentLane create
        self.speed_msg = Float64() # speed msg create
        self.angle_msg = Float64() # angle msg create
        self.angle_ema = Float64()
        self.angle_ema.data = 0.57

        self.initDriveFlag = True

        # slide window return variable initialization
        self.slide_img = None 
        self.slide_x_location = 0
        self.current_lane_window = ""

        # for child sign
        self.is_child_detected = False # child sign 검출 여부
        self.is_child_detecting = False
        self.slow_t1 = 0.0             # 어린이보호구역 주행 시간
        self.sign_data = 0             # child sign id
        self.slow_flag = False         # 저속 주행 시작 시간을 구하기 위한 lock
        self.child_cnt = 0
        self.none_child_cnt = 0
        self.stop_t1 = 0.0
        self.stop_flag = False         # 정지를 시작하는 시간을 구하기 위한 lock
        self.angle_child_before = 0.5

        # for rubbercone misson
        self.is_rubbercone_mission = False # rubber cone 미션 구간 진입 여부
        self.rubbercone_angle_error = 0    # 양옆 rubber cone 좌표 오차

        # for static obstacle
        self.is_doing_static_mission = False
        self.static_t1 = 0.0           # 정적 미션 시작 시간
        self.static_flag = False       # 정적 미션 시간을 구하기 위한 lock

        # for dynamic obstacle
        self.is_doing_dynamic_mission = False
        self.dynamic_t1 = 0.0           # 동적 미션 시작 시간
        self.dynamic_flag = False       # 동적 미션 시간을 구하기 위한 lock
        self.isDynamicMission = False

        self.obstacle_img = []
        self.originalImg = []

        # publisher : mainAlgorithm에서 계산한 속력과 조향각을 전달함
        rospy.Timer(rospy.Duration(1.0/30.0), self.timerCallback)
        self.webot_speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1) # motor speed
        self.webot_angle_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1) # servo angle
        self.webot_lane_pub = rospy.Publisher("/currentLane", Float64, queue_size=1) # 1 = left, 2 = right

        # subscriber : child_sign id, rubber_cone 조향각, 물체 감지 상태를 받아 속력과 조향각을 구함
        rospy.Subscriber("usb_cam/image_rect_color", Image, self.laneCallback)
        rospy.Subscriber("sign_id", Int32, self.child_sign_callback)
        rospy.Subscriber("rubber_cone", Float32, self.rubbercone_callback)
        rospy.Subscriber("lidar_warning", String, self.warning_callback) # lidar 에서 받아온 object 탐지 subscribe (warning / safe)

        self.initDrive_t1 = rospy.get_time()

    def timerCallback(self, _event):
        try:
            self.mainAlgorithm()
            pass
        except:
            pass
    
    def laneCallback(self, _data):
        # detect lane
        if self.initialized == False:
            cv2.namedWindow("Simulator_Image", cv2.WINDOW_NORMAL) 
            cv2.createTrackbar('low_H', 'Simulator_Image', 213, 360, nothing)
            cv2.createTrackbar('low_L', 'Simulator_Image', 110, 255, nothing)
            cv2.createTrackbar('low_S', 'Simulator_Image', 10, 255, nothing)
            cv2.createTrackbar('high_H', 'Simulator_Image', 300, 360, nothing)
            cv2.createTrackbar('high_L', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_S', 'Simulator_Image', 255, 255, nothing)
            self.initialized = True
        
        cv2_image = self.bridge.imgmsg_to_cv2(_data)
        self.originalImg = cv2_image.copy()
        
        if self.evalutator.evaluate(self.originalImg) == True:
            self.isDynamicMission = True
        else:
            self.isDynamicMission = False
        

        cv2.imshow("original", cv2_image) 

        low_H = cv2.getTrackbarPos('low_H', 'Simulator_Image')
        low_L = cv2.getTrackbarPos('low_L', 'Simulator_Image')
        low_S = cv2.getTrackbarPos('low_S', 'Simulator_Image')
        high_H = cv2.getTrackbarPos('high_H', 'Simulator_Image')
        high_L = cv2.getTrackbarPos('high_L', 'Simulator_Image')
        high_S = cv2.getTrackbarPos('high_S', 'Simulator_Image')

        cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HLS) # BGR to HSV

        lower_lane = np.array([low_H, low_L, low_S]) # 
        upper_lane = np.array([high_H, high_L, high_S])

        lane_image = cv2.inRange(cv2_image, lower_lane, upper_lane)

        # cv2.imshow("Lane Image", lane_image)
        self.laneDetection(lane_image)

        cv2.waitKey(1)
    
    def laneDetection(self, lane_image):
        kernel_size = 5
        blur_img = cv2.GaussianBlur(lane_image,(kernel_size, kernel_size), 0)
        warped_img = self.warper.warp(blur_img)
        # cv2.imshow("warped_img", warped_img)
        self.slide_img, self.slide_x_location, self.current_lane_window = self.slidewindow.slidewindow(warped_img)
        cv2.imshow("slide_img", self.slide_img)
        # rospy.loginfo("CURRENT LANE WINDOW: {}".format(self.current_lane_window))


    def child_sign_callback(self, _data):
        # aruco 알고리즘으로 child sign이 검출되었다면 is_child_detected = True
        rospy.loginfo(f"ARUCO sign: {_data.data}")
        if _data.data == 100:
            self.child_cnt += 1
            if self.child_cnt >=10 :
                self.sign_data = _data.data
                self.is_child_detecting = True
                self.child_cnt = 0
        elif _data.data == 0 and self.is_child_detecting == True:
            self.none_child_cnt += 1
            self.is_child_detected = True
            if self.none_child_cnt >= 10:
                self.is_child_detecting = False
                self.none_child_cnt = 0
        else :
            self.sign_data = 0


    def warning_callback(self, _data):
        # lidar에서 장애물을 인식한 후 상태 변수를 갱신함
        # rubber cone이 감지되었을 때
        if self.is_rubbercone_mission == True:
            self.is_rubbercone_mission = True
        # lidar_warning 상태가 safe일 때 상태 변수 갱신
        elif _data.data == "safe":
            if self.is_doing_static_mission:
                self.is_safe = False
            else:
                self.is_safe = True
        # lidar_warning 상태가 WARNING일 때(rubber cone이 아닌 장애물이 한 개 이상 감지됨)
        elif _data.data == "WARNING":
            self.is_safe = False
            rospy.loginfo("WARNING!")
        else:
            pass

    def rubbercone_callback(self, _data):
        self.rubbercone_angle_error = _data.data
        # rubber cone이 감지된 경우
        if self.rubbercone_angle_error < 10.0 :
            self.is_rubbercone_mission = True
        # 감지된 rubber cone이 없는 경우(subscribe 1000.0)
        else :
            self.is_rubbercone_mission = False

    def initDrive(self):
        rospy.loginfo("initDrive")
        self.speed_msg.data = 900
        self.angle_msg.data = 0.57
        initDrive_t2 = rospy.get_time()
        if initDrive_t2 - self.initDrive_t1 >= 1:
            self.initDriveFlag = False

        self.webot_angle_pub.publish(self.angle_msg)
        self.webot_speed_pub.publish(self.speed_msg)
        
    def childProtectDrive(self):
        rospy.loginfo("MISSION: Child Sign")
        # child sign detected, waiting sign to disappear.
        if self.sign_data == 100:
            self.angle_msg.data = (self.slide_x_location - 280) * 0.0035 + 0.5 + 0.07# 조향각 계산
            self.angle_child_before = self.angle_msg.data
            self.speed_msg.data = 2000 # defalut speed
        # sign disappered, drive slow
        elif self.sign_data == 0: 
            t2 = rospy.get_time() # 정지, 저속 주행 시간 counter
            # 미션을 처음 시작하는 경우 시간을 stop_t1으로 저장함
            if self.stop_flag == False:
                self.stop_t1 = rospy.get_time() # start time
                self.is_child_detected = True
                self.stop_flag = True
            # 5초간 정지
            elif t2 - self.stop_t1 <= 5:
                self.speed_msg.data = 0
                self.angle_msg.data = self.angle_child_before
            # 1초간 이전 각도로 주행
            elif t2 - self.stop_t1 <= 6:
                self.angle_msg.data = self.angle_child_before
                self.speed_msg.data = 900
            else:
                # 저속 주행을 처음 시작하는 경우 시간을 slow_t1으로 저장함
                if self.slow_flag == False:
                    self.slow_t1 = rospy.get_time()
                    self.is_child_detected = True
                    self.slow_flag = True
                elif t2 - self.slow_t1 <= 15:
                    self.angle_msg.data = (self.slide_x_location - 280) * 0.003 + 0.5 +0.07
                    self.speed_msg.data = 900
                else:
                    self.stop_flag = False
                    self.slow_flag = False
                    self.is_child_detected = False
    
    def rubberconeDrive(self):
        rospy.loginfo("MISSION: Rubber Cone")
        self.speed_msg.data = 1500
        self.angle_msg.data = (self.rubbercone_angle_error  + 0.5  ) * 1.2      
        rospy.loginfo(f"rubber error: {self.rubbercone_angle_error}")
        self.current_lane = "LEFT"
    
    def obstacleDrive(self):
        rospy.loginfo("MISSION: STATIC")
        t2 = rospy.get_time()
        # 장애물 미션 시작 시간
        if self.static_flag == False:
            self.static_t1 = rospy.get_time()
            self.is_doing_static_mission = True
            self.static_flag = True
        
        if self.current_lane == "LEFT":
            self.speed_msg.data = 1000
            if t2 - self.static_t1 < 2.3:
                self.angle_msg.data = 0.87
            elif t2 - self.static_t1 < 4:
                self.angle_msg.data = 0.27
            else:
                self.is_doing_static_mission = False
                self.angle_msg.data = 0.57
                self.current_lane = "RIGHT"
                self.is_safe = True
        elif self.current_lane == "RIGHT":
            self.speed_msg.data = 1000
            if t2 - self.static_t1 < 2.3:
                self.angle_msg.data = 0.27
            elif t2 - self.static_t1 < 4:
                self.angle_msg.data = 0.87
            else:
                self.is_doing_static_mission = False
                self.angle_msg.data = 0.27
                self.current_lane = "LEFT"
                self.is_safe = True
        
        self.webot_speed_pub.publish(self.speed_msg)
        self.webot_angle_pub.publish(self.angle_msg)

    def dynamicObstacle(self):
        rospy.loginfo("MISSION: Dynamic")
        self.static_flag = False
        self.is_doing_static_mission = False
        self.speed_msg.data = 0
        self.angle_msg.data = 0.57            
        
        self.publish()


    def defaultDrive(self):
        rospy.loginfo("MISSION: Default Driving")
        self.speed_msg.data = 2000 # defalut speed
        self.angle_msg.data = (self.slide_x_location - 280) * 0.003 + 0.57 # 조향각 계산
        
        # 조향범위 제한
        if self.angle_msg.data < 0.1:
            self.angle_msg.data = 0.1


    def mainAlgorithm(self):
        rospy.loginfo("ENTRY")
        if self.current_lane == "LEFT":
            self.lane_msg.data = 1
        elif self.current_lane == "RIGHT":
            self.lane_msg.data = 2
        self.webot_lane_pub.publish(self.lane_msg) # publish current lane to slide window
        rospy.loginfo("MAIN")
        # 0. init drive 
        if self.initDriveFlag == True:
            self.initDrive()
        
        # 1. dynamic obstacle
        elif self.isDynamicMission == True:
            self.dynamicObstacle()

        # 2. child protect driving
        elif self.is_child_detected == True:
            self.childProtectDrive()
            self.publish()

        # 3. rubbercone mission
        elif self.is_rubbercone_mission == True:
            self.rubberconeDrive()
            self.publish()
        
        # 4. obstacle mission
        elif self.is_safe == False:
            self.obstacleDrive()

        # 5. defalut driving
        else:
            self.defaultDrive()
            self.publish()

    def publish(self):
        rospy.loginfo("publish")
        self.angle_ema.data = 0.2 * self.angle_ema.data + 0.8 * self.angle_msg.data

        self.webot_speed_pub.publish(self.speed_msg) # publish speed
        self.webot_angle_pub.publish(self.angle_ema) # publish angle


def nothing(x):
    pass

def run():

    rospy.init_node("main_class_run")
    control = MainLoop()
    rospy.Timer(rospy.Duration(1.0/30.0), control.timerCallback) 
    rospy.spin()

if __name__ == "__main__":
    run()
