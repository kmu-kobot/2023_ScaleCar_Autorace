#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image, LaserScan

class scanDataBridge():
    def __init__(self):
        rospy.Subscriber("scan", LaserScan, self.ridar_callback)

        self.front_pub = rospy.Publisher("/scan/front_scan", LaserScan , queue_size=5)
        self.rear_pub = rospy.Publisher("/scan/rear_scan", LaserScan , queue_size=5)
    
    def ridar_callback(self, _data):
        self.front_pub.publish(_data)
        self.rear_pub.publish(_data)

def run():
    rospy.init_node("scanBridge")
    new_class = scanDataBridge()
    rospy.spin()

if __name__ == '__main__':
    run()
