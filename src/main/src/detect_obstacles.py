#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
import math
import time
from std_msgs.msg import String, Int32, Float32
from obstacle_detector.msg import Obstacles

class LidarReceiver():
    def __init__(self):
        rospy.Subscriber("/raw_obstacles", Obstacles, self.lidar_callback)
        self.warning_pub = rospy.Publisher("lidar_warning", String, queue_size=5)
        self.object_pub = rospy.Publisher("object_condition", Float32, queue_size=5)

        self.count_flag = 0
        self.flag_flag = 0
        self.count_t1 = 0
        self.x1 = 0
        self.x2 = 0

    def lidar_callback(self, _data):
        # ROI
        left_y = -0.2
        right_y = 0.2
        front_x = -1.6
        back_x = 0
        WARNING_CNT = 1

        self.point_cnt = 0
        self.dynamic_cnt = 0

        # 동적장애물 왼쪽 기둥 인식
        for i in _data.circles :
            if left_y < i.center.y < right_y and front_x < i.center.x < back_x:
                self.object_pub.publish(i.center.y)
                
            if left_y < i.center.y < right_y and -1.3 < i.center.x < 0:
                self.point_cnt += 1
                    
            if self.point_cnt >= WARNING_CNT:
                self.warning_pub.publish("WARNING")

            else:
                self.warning_pub.publish("safe")
                self.point_cnt = 0

def run():
    rospy.init_node("lidar_example")
    new_class = LidarReceiver()
    rospy.spin()


if __name__ == '__main__':
    run()