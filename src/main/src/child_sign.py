#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image 
from fiducial_msgs.msg import Fiducial, FiducialArray

class Sign():

    def __init__(self):
        self.sign_id = 0
        # subscriber: FiducialArray를 받아 sign_id를 구함
        rospy.Subscriber("/fiducial_vertices", FiducialArray, self.child_sign_callback)
        # publisher: sign_id를 전달함
        self.sign_id_pub = rospy.Publisher("sign_id", Int32, queue_size=1)
        self.pub_cnt = 0


    def child_sign_callback(self, _data):
        # child sign을 감지하면 감지된 fiducial_id를 publish
        if (len(_data.fiducials) > 0):
            self.sign_id = _data.fiducials[0].fiducial_id
            self.sign_id_pub.publish(self.sign_id)
            self.pub_cnt = 0
        # child sign이 감지되지 않으면 0을 publish
        else:
            self.pub_cnt += 1
            if self.pub_cnt > 20:
                self.sign_id_pub.publish(0)
                self.pub_cnt = 0


def run():
    rospy.init_node("sign_id")
    new_class = Sign()
    rospy.spin()


if __name__ == '__main__':
    run()