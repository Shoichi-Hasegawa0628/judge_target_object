#!/usr/bin/env python
# -*- coding: utf-8 -*-
# YOLOv3で検出した物体から, 識別精度0.5以上の物体のみtarget_objectとして物体の画像領域だけを切り抜くコード
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from sensor_msgs.msg import Image
from judge_target_object.srv import SendImageYOLOv3
from judge_target_object.srv import SendImageYOLOv3Response

class JudgeTargetObjectYOLOv3():
    
    def __init__(self):
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_callback, queue_size=10)
        rospy.Subscriber('/darknet_ros/detection_image', Image, self.yolov3_image_callback, queue_size=10)
        rospy.Service('judge_yolov3', SendImageYOLOv3, self.judge_target_object_yolov3)
        self.cv_bridge = CvBridge()
        self.detect_objects_info = []
        self.detect_object_img = None
        self.processed_object_img = None


    def judge_target_object_yolov3(self, msg):
        object_list = self.detect_objects_info
        observed_img = self.image_ros_to_opencv(msg.rgb_image)
        yolov3_img = self.processed_object_img
        img_num = msg.count

        for i in range(len(object_list)):
            while len(self.detect_objects_info) == 0: 
                pass
            if object_list[i].probability >= 0.5:
                cut_img = observed_img[object_list[i].ymin : object_list[i].ymax, object_list[i].xmin : object_list[i].xmax]
                cut_img_yolov3 = yolov3_img[object_list[i].ymin : object_list[i].ymax, object_list[i].xmin : object_list[i].xmax]
                cv2.imwrite("../data/trimming/{}/trimming_img_{}.jpg".format(img_num, i), cut_img)
                cv2.imwrite("../data/yolov3/{}/yolov3_img_{}.jpg".format(img_num, i), cut_img_yolov3)
                print("OK")

        return SendImageYOLOv3Response(success = True)


    def bounding_callback(self, msg):
        self.detect_objects_info = msg.bounding_boxes

  
    def yolov3_image_callback(self, img):
        self.detect_object_img = img
        self.processed_object_img = self.image_ros_to_opencv(self.detect_object_img)


    def image_ros_to_opencv(self, img):
        try:
            observed_img = img
            observed_img = self.cv_bridge.imgmsg_to_cv2(observed_img, 'passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)

        observed_img = cv2.cvtColor(observed_img, cv2.COLOR_BGR2RGB)
        return observed_img

if __name__ == "__main__":
    rospy.init_node('judge_target_object_yolov3')
    JudgeTargetObjectYOLOv3()
    rospy.spin()