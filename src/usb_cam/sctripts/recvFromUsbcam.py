#!/usr/bin/env python3
# -*-coding:utf-8-*-

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

class img_converter:
    def __init__(self):
        self.canny_pub = rospy.Publisher("/Canny_img2", CompressedImage)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.callback, queue_size =1)

    def callback(self, data):
        print("="*40)
        rospy.loginfo("frame subscribed")
        np_arr = np.frombuffer(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 
        
        Canny_img = cv2.Canny(cv_image, 150, 200)
        cv2.imshow("canny", ~Canny_img)
        cv2.waitKey(1)

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
        self.canny_pub.publish(msg)
        

def run():
    try:
        rospy.init_node('image_converter', anonymous = True)
        ic = img_converter()
    
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("shut down")
        cv2.destroyAllwindows()


if __name__ =="__main__":
    run()
    