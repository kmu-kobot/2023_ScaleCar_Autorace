#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Int32, String, Float32
from sensor_msgs.msg import Image 
from obstacle_detector.msg import Obstacles

class ClusterLidar :

    def __init__(self) :
        rospy.Subscriber("/raw_obstacles", Obstacles, self.rubber_callback)
        self.rabacon_pub = rospy.Publisher("rubber_cone", Float32, queue_size = 5)
        self.angle = 0.0 

    def rubber_callback(self, _data):
        left_rabacon = []
        right_rabacon = []
        for i in _data.circles :
            if -1.7 < i.center.x < 0  :
                if 0 < i.center.y < 1 :
                    left_rabacon.append(i)
                elif -1 < i.center.y < 0 :
                    right_rabacon.append(i)

        if len(left_rabacon) >= 1 and len(right_rabacon) >= 1:
            left_close_rabacon = sorted(left_rabacon, key = lambda x : -x.center.x)[0]
            right_close_rabacon = sorted(right_rabacon, key = lambda x : -x.center.x)[0]
            raba = (left_close_rabacon.center.y + right_close_rabacon.center.y)
            self.rabacon_pub.publish(raba)
        else :
            self.rabacon_pub.publish(1000.0)

def run():
    rospy.init_node("rubber_drive")
    cluster = ClusterLidar()
    rospy.spin()

if __name__ == '__main__':
    run()