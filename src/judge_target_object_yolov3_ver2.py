#!/usr/bin/env python
# -*- coding: utf-8 -*-
# YOLOv3で検出した物体から, 識別精度0.7以上の物体のみtarget_objectの候補として登録するコード
import datetime
import sys

import actionlib
from timeit import default_timer as timer
from sensor_msgs.msg import RegionOfInterest
import rospy
#import numpy as np
import cv2
#from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import CheckForObjectsAction, CheckForObjectsGoal, CheckForObjectsResult
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from judge_target_object.msg import BoundingBox2D
from sensor_msgs.msg import Image


class JudgeTargetObjectYOLOv3():
    
    def __init__(self):
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.detect, queue_size=10)
        self._client = actionlib.SimpleActionClient('/darknet_ros/check_for_objects', CheckForObjectsAction)
        print('Waiting for the server "{}"...'.format("/darknet_ros/check_for_objects"))
        self._client.wait_for_server()
        print('Connected to the service "{}"...'.format("/darknet_ros/check_for_objects"))

        #rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_callback, queue_size=10)
        #rospy.Subscriber('/darknet_ros/detection_image', Image, self.yolov3_image_callback, queue_size=10)
        #self.cv_bridge = CvBridge()
        #self.detect_objects_info = []
        #self.detect_object_img = None
        #self.target_list = []

        #while len(self.detect_objects_info) == 0: #or self.detect_object_img == None:
        #    pass
        #for j in range(1000):
        #    self.judge_target_object_yolov3()


    """
    def judge_target_object_yolov3(self):
        object_list = self.detect_objects_info
        detect_object_img = self.detect_object_img

        for i in range(len(object_list)):
            if object_list[i].probability >= 0.5:
                self.target_list.append(object_list[i])
                
                #mask = np.zeros((int(object_list[i].ymax - object_list[i].ymin), int(object_list[i].xmax - object_list[i].xmin)), dtype = np.uint8)
                #mask = cv2.rectangle(mask, (object_list[i].xmin, object_list[i].ymin), (object_list[i].xmax ,object_list[i].ymax), color=(255,255,255), thickness=2)
                #remove_line_img = cv2.inpaint(detect_object_img, mask, 3, cv2.INPAINT_TELEA)
                #cut_img = remove_line_img[object_list[i].ymin : object_list[i].ymax, object_list[i].xmin : object_list[i].xmax]
                

                #mask = np.zeros((int(cut_img.shape[1]), int(cut_img.shape[0])), dtype = np.uint8)
                #print("size:", cut_img.shape[0], cut_img.shape[1])
                #mask = cv2.rectangle(mask, (object_list[i].xmin, object_list[i].ymin), (object_list[i].xmax ,object_list[i].ymax), color=(255,255,255), thickness=2)
                #remove_line_img = cv2.inpaint(cut_img, mask, 3, cv2.INPAINT_TELEA)
                # remove_line_img = cv2.inpaint(cut_img, mask, 3, cv2.INPAINT_NS)
                cut_img = detect_object_img[object_list[i].ymin : object_list[i].ymax, object_list[i].xmin : object_list[i].xmax]
                h, w = cut_img[:2]
                h1, h2 = int(h * 0.05), int(h * 0.95)
                w1, w2 = int(w * 0.05), int(w * 0.95)
                remove_line_img = cut_img[h1: h2, w1: w2]

                cv2.imwrite("../data/trimming/trimming_img_{}.jpg".format(object_list[i].Class), remove_line_img)
        print("Target_Object:", self.target_list)
        """


    def detect(self, img):
        """
        :type img: Image
        """
        rospy.loginfo("OK")
        goal = CheckForObjectsGoal()
        goal.image = img
        goal.id = int(datetime.datetime.now().strftime("%S%Z%z%f")) % 65535 - 32768

        start = timer()
        # Send request.
        while True:
            
            self._client.send_goal(goal)
            self._client.wait_for_result()
            rospy.loginfo("OKKKKK")
            raw_res = self._client.get_result()
            print(goal.id, raw_res.id)
            if raw_res.id == goal.id:
                break

        # Convert 'raw_data'.
        conversion_res = self._translate_response(raw_res)

        end = timer()
        print('prediction finished. ({} [sec])'.format(end - start))
        sys.stdout.flush()

        cv2.imwrite("../data/trimming/trimming_img.jpg", conversion_res)
        #return conversion_res


    @staticmethod
    def _translate_response(raw_res):
        """
        :type raw_res: CheckForObjectsResult
        """
        bboxes = raw_res.bounding_boxes.bounding_boxes
        conversion_res = [BoundingBox2D() for i in range(len(bboxes))]
        for i, bbox in enumerate(bboxes):
            bounding_box = conversion_res[i]
            roi_param = {
                'x_offset': bbox.xmin,
                'y_offset': bbox.ymin,
                'height': bbox.ymax - bbox.ymin + 1,
                'width': bbox.xmax - bbox.xmin + 1,
                'do_rectify': True
            }
            bounding_box.region = RegionOfInterest(**roi_param)
            bounding_box.name = bbox.Class
            bounding_box.score = bbox.probability
            bounding_box.label = 0

        return conversion_res


    """
    def bounding_callback(self, msg):
        self.detect_objects_info = msg.bounding_boxes

    
    def yolov3_image_callback(self, img):
        self.detect_object_img = img
        self.detect_object_img = self.yolov3_image_ros_to_opencv(self.detect_object_img)
        #cv2.imshow('YOLO', self.detect_object_img)
        #cv2.waitKey(1)


    def yolov3_image_ros_to_opencv(self, img):
        try:
            object_image = img
            object_image = self.cv_bridge.imgmsg_to_cv2(object_image, 'passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)

        object_image = cv2.cvtColor(object_image, cv2.COLOR_BGR2RGB)
        return object_image
    """

if __name__ == "__main__":
    rospy.init_node('judge_target_object_yolov3')
    JudgeTargetObjectYOLOv3()
    rospy.spin()