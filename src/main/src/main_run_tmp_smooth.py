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

class MainLoop:
    
    def __init__(self):
        self.warper = Warper()
        self.slidewindow = SlideWindow()
        self.bridge = CvBridge()

        self.current_lane = "LEFT"
        self.is_safe = True
        self.initialized = False

        self.lane_msg = Float64() # currentLane create
        self.speed_msg = Float64() # speed msg create
        self.angle_msg = Float64() # angle msg create
        self.angle_ema = Float64()
        self.angle_ema.data =0.5

        # slide window return variable initialization
        self.slide_img = None 
        self.slide_x_location = 0
        self.current_lane_window = ""

        # for child sign
        self.is_child_detected = False # child sign 검출 여부
        self.slow_t1 = 0.0             # 어린이보호구역 주행 시간
        self.sign_data = 0             # child sign id
        self.slow_flag = 0             # 저속 주행 시작 시간을 구하기 위한 lock
        self.child_cnt = 0
        self.stop_t1 = 0.0
        self.stop_flag = 0             # 정지를 시작하는 시간을 구하기 위한 lock

        # for rubbercone misson
        self.is_rubbercone_mission = False # rubber cone 미션 구간 진입 여부
        self.rubbercone_angle_error = 0    # 양옆 rubber cone 좌표 오차

        # for static obstacle
        self.is_static_mission = False # 정적 장애물 인식 여부
        self.turn_left_flag = False
        self.turn_right_flag = False

        # for dynamic obstacle
        self.is_dynamic_mission = False # 동적 장애물 인식 여부

        self.obstacle_img = []

        self.y_list = []          # 감지된 장애물 좌표 리스트
        self.y_list_sort = []     # 감지된 장애물 왼쪽부터 나열한 리스트
        self.dynamic_obs_cnt = 0  # 동적 장애물 개수
        self.static_cnt = 0

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
        rospy.Subscriber("object_condition", Float32, self.object_callback) 

        self.initDrive_t1 = rospy.get_time()

    def timerCallback(self, _event):
        try:
            self.mainAlgorithm()
            # pass
        except:
            pass
    
    def laneCallback(self, _data):
        # detect lane
        if self.initialized == False:
            cv2.namedWindow("Simulator_Image", cv2.WINDOW_NORMAL) 
            cv2.createTrackbar('low_H', 'Simulator_Image', 23, 360, nothing)
            cv2.createTrackbar('low_L', 'Simulator_Image', 23, 255, nothing)
            cv2.createTrackbar('low_S', 'Simulator_Image', 10, 255, nothing)
            cv2.createTrackbar('high_H', 'Simulator_Image', 300, 360, nothing)
            cv2.createTrackbar('high_L', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_S', 'Simulator_Image', 255, 255, nothing)
            self.initialized = True
        
        cv2_image = self.bridge.imgmsg_to_cv2(_data)
        self.obstacle_img = cv2_image

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
        # aruco 알고리즘으로 child sign이 검출되었다면 is_child_detected = True
        rospy.loginfo(f"ARUCO sign: {_data.data}")
        if _data.data == 1:
            self.child_cnt += 1
            if self.child_cnt >=20 :
                self.sign_data = _data.data
                self.is_child_detected = True
                self.child_cnt = 0
        else :
            self.sign_data = 0


    def warning_callback(self, _data):
        # lidar에서 장애물을 인식한 후 상태 변수를 갱신함
        # rubber cone이 감지되었을 때
        if self.is_rubbercone_mission == True:
            self.is_rubbercone_mission = True
        # lidar_warning 상태가 safe일 때
        elif _data.data == "safe":
            self.is_safe = True
            self.y_list = []
            if self.is_dynamic_mission == True:
                self.dynamic_obs_cnt += 1
                if self.dynamic_obs_cnt >= 50 : # 50회 이상 감지되면
                    self.is_dynamic_mission = False
                    self.dynamic_obs_cnt = 0
            self.is_static_mission = False
        # lidar_warning 상태가 WARNING일 때(rubber cone이 아닌 장애물)
        elif _data.data == "WARNING":
            self.is_safe = False
            rospy.loginfo("WARNING!")
        else:
            pass


    def object_callback(self, _data):
        # 
        if self.is_dynamic_mission == False or len(self.y_list) <= 19:
            self.y_list.append(_data.data)
            if len(self.y_list) >= 21 :
                del self.y_list[0]
        else:
            rospy.logwarn("Unknown warning state!")


    def rubbercone_callback(self, _data):
        self.rubbercone_angle_error = _data.data
        # rubber cone이 감지된 경우
        if self.rubbercone_angle_error < 10.0 :
            self.is_rubbercone_mission = True
            self.is_dynamic_mission = False
            self.is_static_mission = False
            self.static_cnt = 0
            self.dynamic_obs_cnt = 0
            self.turn_left_flag = False
            self.turn_right_flag = False
        # 감지된 rubber cone이 없는 경우(subscribe 1000.0)
        else :
            self.is_rubbercone_mission = False


    def mainAlgorithm(self):

        if self.current_lane == "LEFT":
            self.lane_msg = 1
        elif self.current_lane == "RIGHT":
            self.lane_msg = 2
        self.webot_lane_pub.publish(self.lane_msg) # publish current lane to slide window

        # 0. init drive 
        initDrive_t2 = rospy.get_time()
        if initDrive_t2 - self.initDrive_t1 <= 1:
            self.speed_msg.data = 900
            self.angle_msg.data = 0.5

        # 1. child protect driving
        elif self.is_child_detected == True:
            rospy.loginfo("MISSION: Child Sign")
            # child sign detected, waiting sign to disappear.
            if self.sign_data == 1:
                self.angle_msg.data = (self.slide_x_location - 280) * 0.0035 + 0.5 # 조향각 계산
                self.speed_msg.data = 2000 # defalut speed

            # sign disappered, drive slow
            elif self.sign_data == 0: 
                t2 = rospy.get_time() # 정지, 저속 주행 시간 counter
                # 미션을 처음 시작하는 경우 시간을 stop_t1으로 저장함
                if self.stop_flag == 0:
                    self.stop_t1 = rospy.get_time() # start time
                    self.is_child_detected = True
                    self.stop_flag = 1
                # 5초간 정지
                elif t2 - self.stop_t1 <= 5:
                    self.speed_msg.data = 0
                    self.angle_msg.data = 0.5
                # 5초간 정지를 완료하고 저속 주행을 시작할 때
                else:
                    # 저속 주행을 처음 시작하는 경우 시간을 slow_t1으로 저장함
                    if self.slow_flag == 0:
                        self.slow_t1 = rospy.get_time()
                        self.is_child_detected = True
                        self.slow_flag = 1
                    elif t2 - self.slow_t1 <= 15:
                        self.angle_msg.data = (self.slide_x_location - 280) * 0.003 + 0.5
                        self.speed_msg.data = 900
                    else:
                        self.stop_flag = 0
                        self.slow_flag = 0
                        self.is_child_detected = False

        # 2. rubbercone mission
        elif self.is_rubbercone_mission == True:
            rospy.loginfo("MISSION: Rubber Cone")
            self.speed_msg.data = 1500
            self.angle_msg.data = (self.rubbercone_angle_error  + 0.5  ) * 1.2      
            rospy.loginfo(f"rubber error: {self.rubbercone_angle_error}")
            self.current_lane = "LEFT"
        
        # 미완
        # 3. obstacle mission
        elif self.is_safe == False:
           

        # 4. defalut driving
        else:
            rospy.loginfo("MISSION: Default Driving")
            # rospy.loginfo(f"slide: {self.slide_x_location}")
            self.speed_msg.data = 2000 # defalut speed
            self.angle_msg.data = (self.slide_x_location - 280) * 0.003 + 0.5 # 조향각 계산
            
            # 조향범위 제한
            if self.angle_msg.data < 0.2:
                self.angle_msg.data = 0.2
            elif self.angle_msg > 0.8:
                self.angle_msg.data = 0.8

        # trim data before publish
        self.angle_ema = 0.3 * self.angle_ema + 0.7 * self.angle_msg

        self.webot_speed_pub.publish(self.speed_msg) # publish speed
        self.webot_angle_pub.publish(self.angle_ema) # publish angle

    def change_line_left(self) :
        self.speed_msg.data = 1000.0
        self.angle_msg.data = 0.8
        self.webot_speed_pub.publish(self.speed_msg)
        self.webot_angle_pub.publish(self.angle_msg)

    def change_line_right(self) :
        self.speed_msg.data = 1000.0
        self.angle_msg.data = 0.2
        self.webot_speed_pub.publish(self.speed_msg)
        self.webot_angle_pub.publish(self.angle_msg)

    def stop(self):
        os.system("clear")
        self.speed_msg.data = 0.0
        self.angle_msg.data = 0.5
        rospy.loginfo("STOP")
        

def nothing(x):
    pass

def run():

    rospy.init_node("main_class_run")
    control = MainLoop()
    rospy.Timer(rospy.Duration(1.0/30.0), control.timerCallback) 
    rospy.spin()

if __name__ == "__main__":
    run()
