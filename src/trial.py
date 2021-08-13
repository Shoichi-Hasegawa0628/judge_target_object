#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox

class Trial():
    def __init__(self):
        self.img_pub = rospy.Publisher('/object_image', Image, queue_size=1) # YOLOv3に画像を配信
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_callback, queue_size=1)
        self.cv_bridge = CvBridge()
        self.detect_objects_info = []
        

    def sending_img(self):
        img = cv2.imread('../data/observation/object_image_0.jpg')
        img = self.cv_bridge.cv2_to_imgmsg(img, encoding = "bgr8")
        while len(self.detect_objects_info) == 0:
            #print(len(self.detect_objects_info))
            self.img_pub.publish(img)
        print("finish")


    def bounding_callback(self, msg):
        #print("OK")
        self.detect_objects_info = msg.bounding_boxes
        print(len(self.detect_objects_info))


if __name__ == "__main__":
    rospy.init_node('get_object_image_yolov3')
    trial = Trial()
    trial.sending_img()
    #rospy.spin()
   



"""
img = cv2.imread('../data/trimming/trimming_img_chair.jpg')
#cv2.imshow('color', img)
#cv2.waitKey(0)

h, w = img.shape[:2]
h1, h2 = int(h * 0.05), int(h * 0.95)
w1, w2 = int(w * 0.05), int(w * 0.95)
remove_line_img = img[h1: h2, w1: w2]
cv2.imshow('color', remove_line_img)
cv2.waitKey(0)
"""
