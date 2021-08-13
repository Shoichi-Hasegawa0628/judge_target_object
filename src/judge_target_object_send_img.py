#!/usr/bin/env python
# -*- coding: utf-8 -*-
# judge_yolov3.pyとjudge_mlda.pyに撮影した画像を1枚ずつ送るコード

import rospy
import cv2
from cv_bridge import CvBridge
import time
import glob
import os
from judge_target_object.srv import SendImage
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox

class SendObjectImage():

    def __init__(self):
        # YOLOv3
        self.img_pub = rospy.Publisher('/object_image', Image, queue_size=1)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_callback, queue_size=1)

        # MLDA
        #self.cut_img_pub = rospy.Publisher('/cut_object_image', Image, queue_size=10)
        
        self.cv_bridge = CvBridge()
        self.detect_objects_info = []
        self.sending_image_judge_yolov3()
        #self.sending_image_judge_mlda()


    def sending_image_judge_yolov3(self):
        count = 0
        files = glob.glob("../data/observation/*")
        rospy.loginfo('waiting')
        rospy.wait_for_service('judge_yolov3')
        
        while count != len(files):
                img = cv2.imread('../data/observation/object_image_{}.jpg'.format(count))
                img = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
                while len(self.detect_objects_info) == 0:
                    self.img_pub.publish(img)
                send_img = rospy.ServiceProxy('judge_yolov3', SendImage)
                rgb_image = img
                response = send_img(rgb_image, count)
                print(response)
                time.sleep(1.0)
                count += 1
        
    """
    def sending_image_judge_mlda(self):
        count = 0
        folders = []
        files_list = []
        for i in os.listdir('../data/'):
            if os.path.isdir('../data/' + i):
                folders.append(i)

        for j in range(len(folders)):
            files = glob.glob("../data/trimming/{}/*".format(j))
            files_list.append(files)
        rospy.loginfo('waiting')
        rospy.wait_for_service('judge_mlda')
        
        for k in range(len(folders)):
            while count != int(files_list[k]):
                    img = cv2.imread('../data/trimming/{}/trimming_img_{}.jpg'.format(k, count))
                    img = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
                    while len(self.detect_objects_info) == 0:
                        self.img_pub.publish(img)
                    send_img = rospy.ServiceProxy('judge_mlda', SendImage)
                    rgb_image = img
                    response = send_img(rgb_image, count)
                    print(response)
                    time.sleep(1.0)
                    count += 1
            count = 0
    """

    def bounding_callback(self, msg):
        #print("OK")
        self.detect_objects_info = msg.bounding_boxes
        #print(len(self.detect_objects_info))

if __name__ == "__main__":
    rospy.init_node('send_object_image')
    SendObjectImage()
    rospy.spin()