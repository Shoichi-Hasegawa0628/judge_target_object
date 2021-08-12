#!/usr/bin/env python
# -*- coding: utf-8 -*-
# YOLOv3で検出した物体から, 識別精度0.7以上の物体のみtarget_objectの候補として登録するコード
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from sensor_msgs.msg import Image


class JudgeTargetObjectYOLOv3():
    
    def __init__(self):
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_callback, queue_size=10)
        #rospy.Subscriber('/darknet_ros/detection_image', Image, self.yolov3_image_callback, queue_size=10)
        rospy.Subscriber('/object_image', Image, self.image_callback, queue_size=10)
        self.cv_bridge = CvBridge()
        self.detect_objects_info = []
        self.img = None
        #self.detect_object_img = None
        self.processed_object_img = None
        self.target_list = []

        while len(self.detect_objects_info) == 0: #or self.detect_object_img == None:
            pass
        for j in range(1000):
            self.judge_target_object_yolov3()


    def judge_target_object_yolov3(self):
        object_list = self.detect_objects_info
        #detect_object_img = self.processed_object_img
        detect_object_img = self.processed_object_img

        for i in range(len(object_list)):
            if object_list[i].probability >= 0.5:
                self.target_list.append(object_list[i])
                cut_img = detect_object_img[object_list[i].ymin : object_list[i].ymax, object_list[i].xmin : object_list[i].xmax]
                h, w = cut_img.shape[:2]
                h1, h2 = int(h * 0.05), int(h * 0.95)
                w1, w2 = int(w * 0.05), int(w * 0.95)
                remove_line_img = cut_img[h1: h2, w1: w2]
                
                cv2.imwrite("../data/trimming/trimming_img_{}_{}.jpg".format(i, object_list[i].Class), cut_img)
                cv2.imwrite("../data/remove_line/remove_line_img_{}_{}.jpg".format(i, object_list[i].Class), remove_line_img)


    def bounding_callback(self, msg):
        self.detect_objects_info = msg.bounding_boxes

    """
    def yolov3_image_callback(self, img):
        self.detect_object_img = img
        self.processed_object_img = self.yolov3_image_ros_to_opencv(self.detect_object_img)
        #cv2.imshow('YOLO', self.detect_object_img)
        #cv2.waitKey(1)
    """

    def image_callback(self, img):
        self.img = img
        self.processed_object_img = self.yolov3_image_ros_to_opencv(self.img)


    def yolov3_image_ros_to_opencv(self, img):
        try:
            object_image = img
            object_image = self.cv_bridge.imgmsg_to_cv2(object_image, 'passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)

        object_image = cv2.cvtColor(object_image, cv2.COLOR_BGR2RGB)
        return object_image


if __name__ == "__main__":
    rospy.init_node('judge_target_object_yolov3')
    JudgeTargetObjectYOLOv3()
    rospy.spin()