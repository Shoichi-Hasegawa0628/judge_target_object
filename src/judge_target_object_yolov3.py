#!/usr/bin/env python
# -*- coding: utf-8 -*-
# YOLOv3で検出した物体から, 識別精度0.7以上の物体のみtarget_objectの候補として登録するコード
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from sensor_msgs.msg import Image


class JudgeTargetObjectYOLOv3():
    
    def __init__(self):
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_callback, queue_size=10)
        rospy.Subscriber('/darknet_ros/detection_image', Image, self.yolov3_image_callback, queue_size=10)
        self.cv_bridge = CvBridge()
        self.detect_objects_info = 0
        self.detect_object_img = 0
        self.object_list = {}
        self.target_list = {}
        self.judge_target_object_yolov3()


    def judge_target_object_yolov3(self):
        object_list = self.object_list
        detect_object_img = self.detect_object_img

        for key in object_list:
            if object_list[key] >= 0.7:
                self.target_list [key] = object_list[key]

        print("Target_Object:", self.target_list)
        pass


    def bounding_callback(self, msg):
        self.detect_objects_info = msg.bounding_boxes
        #print(type(self.target_object))
        #print(len(self.target_object))
        #print("LIST[0]", self.detect_objects[0])
        #print("****************************************")
        #print("LIST[1]", self.detect_objects[1])
        #print("****************************************")
        #target_objects = self.detect_objects[0]
        #print(target_objects.probability)

        for i in range (len(self.detect_objects_info)):
            self.object_list ['{}'.format(self.detect_objects_info[i].Class)] = self.detect_objects_info[i].probability
        #print(self.object_list)

    
    def yolov3_image_callback(self, img):
        self.detect_object_img = img
        self.detect_object_img = self.yolov3_image_ros_to_opencv(self.detect_object_img)
        #cv2.imshow('YOLO', self.detect_object_img)
        #cv2.waitKey(1)


    def yolov3_image_ros_to_opencv(self, img):
        try:
            object_image = img
            #print(type(object_image))
            object_image = self.cv_bridge.imgmsg_to_cv2(object_image, 'passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)

        object_image = cv2.cvtColor(object_image, cv2.COLOR_BGR2RGB)
        return object_image


if __name__ == "__main__":
    rospy.init_node('judge_target_object_yolov3')
    JudgeTargetObjectYOLOv3()
    rospy.spin()