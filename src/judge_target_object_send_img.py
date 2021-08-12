#!/usr/bin/env python
# -*- coding: utf-8 -*-
# judge_yolov3.pyとjudge_mlda.pyに画像を1枚ずつ送るコード

import rospy
import cv2
from cv_bridge import CvBridge
import time
import glob
from std_srvs.srv import Empty
from sensor_msgs.msg import Image

class SendObjectImage():

    def __init__(self):
        self.img_pub = rospy.Publisher('/object_image', Image, queue_size=10) # YOLOv3に画像を配信
        self.cut_img_pub = rospy.Publisher('/cut_object_image', Image, queue_size=10) # MLDAに画像を配信
        self.cv_bridge = CvBridge()
        self.sending_image_judge_yolov3()
        #self.sending_image_judge_mlda()


    def sending_image_judge_yolov3(self):
        count = 0
        files = glob.glob("../data/observation/*")
        rospy.loginfo('waiting')
        rospy.wait_for_service('judge_yolov3')

        while count != len(files):
            try:
                img = cv2.imread('../data/observation/object_image_{}.jpg'.format(count))
                img = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
                self.img_pub(img)
                service = rospy.ServiceProxy('judge_yolov3', Empty)
                response = service()
            except rospy.ServiceException as e:
                print("Service call failed: %s", e)
            time.sleep(1.0)
            count += 1

    """
    def sending_image_judge_mlda(self):
        count = 0
        files = glob.glob("../data/remove_line/*")
        rospy.loginfo('waiting')
        rospy.wait_for_service('judge_mlda')

        while count != len(files):
            try:
                img = cv2.imread('../data/remove_line/remove_line_image_{}.jpg'.format(count))
                img = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
                self.cut_img_pub(img)
                service = rospy.ServiceProxy('judge_mlda', Empty)
                response = service()
            except rospy.ServiceException as e:
                print("Service call failed: %s", e)
            time.sleep(1.0)
            count += 1
    """

if __name__ == "__main__":
    rospy.init_node('send_object_image')
    SendObjectImage()