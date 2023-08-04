#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Int32, String, Float32
from sensor_msgs.msg import Image 
from obstacle_detector.msg import Obstacles

class ClusterLidar :

    def __init__(self) :
        rospy.Subscriber("/raw_obstacles", Obstacles, self.rabacon)
        self.rabacon_pub = rospy.Publisher("rubbder_drive", Float32, queue_size = 5)
        self.angle = 0.0 

    def rubber_callback(self, _data):
        pass

def run():
    rospy.init_node("rubber_drive")
    cluster = ClusterLidar()
    rospy.spin()

if __name__ == '__main__':
    run()