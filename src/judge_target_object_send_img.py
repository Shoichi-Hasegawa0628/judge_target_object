#!/usr/bin/env python
# -*- coding: utf-8 -*-
# judge_yolov3.pyとjudge_mlda.pyに撮影した画像を1枚ずつ送るコード

import rospy
import cv2
from cv_bridge import CvBridge
import time
import glob
import os
from judge_target_object.srv import SendImageYOLOv3
from judge_target_object.srv import SendImageMLDA
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from subprocess import * 
from natsort import natsorted

class SendObjectImage():

    def __init__(self):
        # YOLOv3
        self.img_pub = rospy.Publisher('/object_image', Image, queue_size=1)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_callback, queue_size=1)
        
        self.cv_bridge = CvBridge()
        self.detect_objects_info = []
        #self.sending_image_judge_yolov3()
        self.sending_image_judge_mlda()


    def sending_image_judge_yolov3(self):
        time.sleep(5.0)
        count = 0
        files = glob.glob("../data/observation/*")
        rospy.loginfo('waiting')
        rospy.wait_for_service('judge_yolov3')
        
        while count != len(files):
            status = True
            #print(len(files))
            #if count != 0:
            #    self.img_pub.register()
            img = cv2.imread('../data/observation/object_image_{}.jpg'.format(count))
            img = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")

            if count != 0:
                self.img_pub.publish(img)

            base_time = rospy.Time.now().to_sec()
            while len(self.detect_objects_info) == 0: #####ここが問題かも？
                self.img_pub.publish(img)
                if rospy.Time.now().to_sec() - base_time > 10:
                    status = False
                    break
           
            if status is False:
                continue
            
            print("Publish image")
            send_img = rospy.ServiceProxy('judge_yolov3', SendImageYOLOv3)
            rgb_image = img
            response = send_img(rgb_image, count)
            print(response)
            #self.img_pub.unregister()
            #time.sleep(1.0)
            count += 1
            #self.kill_node('send_object_image')
            #rospy.init_node('send_object_image')
              

    
    def sending_image_judge_mlda(self):
        folders = []
        files_list = []
        for i in os.listdir('../data/resize/'):
            #print(os.listdir('../data/resize/' + i))
            if os.path.isdir('../data/resize/' + i):
                folders.append(i)
        print(folders)
        ar_folders = natsorted(folders)
        print(natsorted(folders))

        for j in range(len(ar_folders)):
            files = glob.glob("../data/resize/{}/*".format(int(ar_folders[j])))
            files_list.append(files)
        rospy.loginfo('waiting')
        rospy.wait_for_service('judge_mlda')
        print(files_list)
        
        for k in range(len(ar_folders)):
            for l in range(len(files_list[k])):
                #print(len(files_list[k]))
                count = l
                img = cv2.imread('../data/resize/{}/resize_img_{}.jpg'.format(int(ar_folders[k]), l))
                img = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")

                send_img = rospy.ServiceProxy('judge_mlda', SendImageMLDA)
                yolov3_image = img
                status = "estimate"
                observed_img_idx = int(ar_folders[k])
                response = send_img(yolov3_image, status, observed_img_idx, count)
                print(response)
                print(int(ar_folders[k]), l)

                #time.sleep(1.0)
                #count += 1
            #count = 0


    def bounding_callback(self, msg):
        #print("OK")
        self.detect_objects_info = msg.bounding_boxes
        #print(len(self.detect_objects_info))


    def kill_node(self, nodename): 
        p2 = Popen(['rosnode', 'list'], stdout = PIPE) 
        p2.wait() 
        nodelist = p2.communicate() 
        nd = nodelist[0] 
        nd = nd.split("\n") 
        for i in range(len(nd)): 
            tmp = nd[i] 
            ind = tmp.find(nodename) 
            if ind == 1: 
                call(['rosnode', 'kill', nd[i]]) 
                break 

if __name__ == "__main__":
    rospy.init_node('send_object_image')
    SendObjectImage()
    rospy.spin()