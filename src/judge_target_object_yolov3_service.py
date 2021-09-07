#!/usr/bin/env python
# -*- coding: utf-8 -*-
# YOLOv3で検出した物体から, 識別精度0.5以上の物体のみtarget_objectとして物体の画像領域だけを切り抜くコード
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from sensor_msgs.msg import Image
from judge_target_object.srv import SendImageYOLOv3
from judge_target_object.srv import SendImageYOLOv3Response
import os
from subprocess import * 
import time

class JudgeTargetObjectYOLOv3():
    
    def __init__(self):
        #rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_callback, queue_size=1)
        #rospy.Subscriber('/darknet_ros/detection_image', Image, self.yolov3_image_callback, queue_size=1)
        rospy.Service('judge_yolov3', SendImageYOLOv3, self.judge_target_object_yolov3)
        self.cv_bridge = CvBridge()
        self.detect_objects_info = []
        self.detect_pre_img = 0


    def judge_target_object_yolov3(self, msg):
        time.sleep(3.0)
        try:
            bb = rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes, timeout=10)
        except:
            return SendImageYOLOv3Response(success = False)

        time.sleep(3.0)
        self.detect_objects_info = bb.bounding_boxes
        
        img = 0
        if msg.count != 0:
            while True:
                print("O")
                img = rospy.wait_for_message('/darknet_ros/detection_image', Image, timeout=15) #原因？
                img = self.image_ros_to_opencv(img)
                status = np.array_equal(img, self.detect_pre_img)
                if status is False:
                    break
        
        else:
            print("OO")
            img = rospy.wait_for_message('/darknet_ros/detection_image', Image, timeout=15) #原因？
            img = self.image_ros_to_opencv(img)
        self.detect_pre_img = img
        
        object_list = self.detect_objects_info
        observed_img = self.image_ros_to_opencv(msg.rgb_image)
        #cv2.imshow('color', img)
        #cv2.waitKey(3000)
        img_num = msg.count

        for i in range(len(object_list)):
            while len(self.detect_objects_info) == 0: 
                print("OOO")
                continue
            time.sleep(3.0)
            yolov3_img = img
            if object_list[i].probability >= 0.5:

                height_o = observed_img.shape[0]
                width_o = observed_img.shape[1]

                cut_img = observed_img[object_list[i].ymin : object_list[i].ymax, object_list[i].xmin : object_list[i].xmax]
                cut_img_yolov3 = yolov3_img[object_list[i].ymin : object_list[i].ymax, object_list[i].xmin : object_list[i].xmax]
                #cut_img_yolov3 = cv2.cvtColor(cut_img_yolov3, cv2.COLOR_BGR2RGB)
                cut_img_resize = cv2.resize(cut_img , (int(width_o), int(height_o))) 

                if os.path.exists("/root/RULO/catkin_ws/src/judge_target_object/data/trimming/{}".format(img_num)) is True:
                    pass
                else:
                    os.mkdir("/root/RULO/catkin_ws/src/judge_target_object/data/trimming/{}".format(img_num))

                if os.path.exists("/root/RULO/catkin_ws/src/judge_target_object/data/yolov3/{}".format(img_num)) is True:
                    pass
                else:
                    os.mkdir("/root/RULO/catkin_ws/src/judge_target_object/data/yolov3/{}".format(img_num))

                if os.path.exists("/root/RULO/catkin_ws/src/judge_target_object/data/resize/{}".format(img_num)) is True:
                    pass
                else:
                    os.mkdir("/root/RULO/catkin_ws/src/judge_target_object/data/resize/{}".format(img_num))

                cv2.imwrite("/root/RULO/catkin_ws/src/judge_target_object/data/trimming/{}/trimming_img_{}.jpg".format(img_num, i), cut_img)
                cv2.imwrite("/root/RULO/catkin_ws/src/judge_target_object/data/yolov3/{}/yolov3_img_{}.jpg".format(img_num, i), cut_img_yolov3)
                cv2.imwrite("/root/RULO/catkin_ws/src/judge_target_object/data/resize/{}/resize_img_{}.jpg".format(img_num, i), cut_img_resize)

                cv2.imshow('color', cut_img)
                cv2.waitKey(3000)

        print("OK")
        #img = 0
        #self.processed_object_img = 0
        return SendImageYOLOv3Response(success = True)


    #def bounding_callback(self, msg):
    #    self.detect_objects_info = msg.bounding_boxes

  
    #def yolov3_image_callback(self, img):
        #self.detect_object_img = img
    #    self.processed_object_img = self.image_ros_to_opencv(img)


    def image_ros_to_opencv(self, img):
        try:
            observed_img = img
            observed_img = self.cv_bridge.imgmsg_to_cv2(observed_img, 'passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)

        #observed_img = cv2.cvtColor(observed_img, cv2.COLOR_BGR2RGB)
        return observed_img



if __name__ == "__main__":
    rospy.init_node('judge_target_object_yolov3')
    JudgeTargetObjectYOLOv3()
    rospy.spin()