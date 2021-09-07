#!/usr/bin/env python
# -*- coding: utf-8 -*-
# judge_yolov3.pyとjudge_mlda.pyに撮影した画像を1枚ずつ送るコード

# 必要なライブラリ
# pip install natsort

import rospy
import cv2
from cv_bridge import CvBridge
import time
import glob
import os
from judge_target_object.srv import SendImageYOLOv3
#from judge_target_object.srv import SendImageMLDA
from sensor_msgs.msg import Image
from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from subprocess import * 
from natsort import natsorted
import mlda_ros_main
import shutil


class SendObjectImage():
    def __init__(self):
        self.mlda_main = mlda_ros_main.MLDAMain()
        # YOLOv3
        self.img_pub = rospy.Publisher('/object_image', Image, queue_size=1)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_callback, queue_size=1)
        
        self.cv_bridge = CvBridge()
        self.detect_objects_info = []
        self.sending_image_judge_yolov3()
        result = self.sending_image_judge_mlda()
        
        if result is True:
            print("************************")
            print("************************\n")
            print("Target Object is Found!\n")
            print("************************")
            print("************************")
        
        else:
            print("************************")
            print("************************\n")
            print("Not Found in this place\n")
            print("************************")
            print("************************")
        
        """
        # 使用した画像の保存とディレクトリの初期化
        shutil.copytree("/root/RULO/catkin_ws/src/judge_target_object/data/", "/root/RULO/catkin_ws/src/judge_target_object/result/data") 
        shutil.rmtree("/root/RULO/catkin_ws/src/judge_target_object/data/observation/")
        shutil.rmtree("/root/RULO/catkin_ws/src/judge_target_object/data/resize/")
        shutil.rmtree("/root/RULO/catkin_ws/src/judge_target_object/data/trimming/")
        shutil.rmtree("/root/RULO/catkin_ws/src/judge_target_object/data/yolov3/")
        os.mkdir("/root/RULO/catkin_ws/src/judge_target_object/data/observation")
        os.mkdir("/root/RULO/catkin_ws/src/judge_target_object/data/resize")
        os.mkdir("/root/RULO/catkin_ws/src/judge_target_object/data/trimming")
        os.mkdir("/root/RULO/catkin_ws/src/judge_target_object/data/yolov3")
        """
        
        

    def sending_image_judge_yolov3(self):
        time.sleep(5.0)
        count = 0
        files = glob.glob("/root/RULO/catkin_ws/src/judge_target_object/data/observation/*")
        rospy.loginfo('waiting') 
        rospy.wait_for_service('judge_yolov3') 
        
        while count != len(files):
            status = True
            #print(len(files))
            #if count != 0:
            #    self.img_pub.register()
            img = cv2.imread('/root/RULO/catkin_ws/src/judge_target_object/data/observation/object_image_{}.jpg'.format(count))
            img = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")

            if count != 0:
                self.img_pub.publish(img)

            base_time = rospy.Time.now().to_sec()
            while len(self.detect_objects_info) == 0: 
                self.img_pub.publish(img)
                if rospy.Time.now().to_sec() - base_time > 10:
                    status = False
                    break
           
            if status is False:
                continue
            
            #print("Publish image")
            send_img = rospy.ServiceProxy('judge_yolov3', SendImageYOLOv3)
            rgb_image = img
            response = send_img(rgb_image, count)
            #print(response)
            #self.img_pub.unregister()
            #time.sleep(1.0)
            count += 1
            #self.kill_node('send_object_image')
            #rospy.init_node('send_object_image')
              

    
    def sending_image_judge_mlda(self):
        folders = []
        files_list = []
        for i in os.listdir('/root/RULO/catkin_ws/src/judge_target_object/data/resize/'):
            #print(os.listdir('../data/resize/' + i))
            if os.path.isdir('/root/RULO/catkin_ws/src/judge_target_object/data/resize/' + i):
                folders.append(i)
        #print(folders)
        ar_folders = natsorted(folders)
        #print(natsorted(folders))

        for j in range(len(ar_folders)):
            files = glob.glob("/root/RULO/catkin_ws/src/judge_target_object/data/resize/{}/*".format(int(ar_folders[j])))
            files_list.append(files)
        ar_files_list = natsorted(files_list)
        rospy.loginfo('waiting')
        #print("OOO")
        #rospy.wait_for_service('judge_mlda')
        #print(files_list)
        
        for k in range(len(ar_folders)): 
            #print(k)
            for l in range(len(ar_files_list[k])):
                #print(l)
                #print("okok")
                count = l
                file_names = []
                for i in os.listdir('/root/RULO/catkin_ws/src/judge_target_object/data/resize/{}'.format(int(ar_folders[k]))):
                    #print(i)
                    #print(os.listdir('../data/resize/' + i))
                    #if os.path.isdir('../data/resize/' + i):
                    file_names.append(i)
                #print(file_names)
                #print("*******************************************************************************************\n")
                print("Observed Image Number : " + str(int(ar_folders[k])))
                print("Detected Object :" + file_names[l])

                try:
                    img = cv2.imread('/root/RULO/catkin_ws/src/judge_target_object/data/resize/{}/resize_img_{}.jpg'.format(int(ar_folders[k]), l))
                    cv2.imshow("target_image", img)
                    cv2.waitKey(3000)
                except cv2.error:
                    continue

                #print(type(img))
                try:
                    img = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
                except TypeError:
                    print("Failed: Reading Image")
                    continue

                #send_img = rospy.ServiceProxy('judge_mlda', SendImageMLDA)
                yolov3_image = img
                status = "estimate"
                observed_img_idx = int(ar_folders[k])
                status, object_word, object_prob = self.mlda_main.judge_target_object_mlda (yolov3_image, status, observed_img_idx, count)
                if status is False:
                    continue
                #response = send_img(yolov3_image, status, observed_img_idx, count)
                #print(response)
                print("Status:{}\nPicture:{}\nObject:{}\nObject_Word:{}\nObject_Prob(%):{}\n".format(status, int(ar_folders[k]), l, object_word, object_prob))
               
                #print(int(ar_folders[k]), l)

                # 対象物が存在するかの判断
                word = rospy.wait_for_message("/human_command", String, timeout=None)
                command_object = word.data

                if command_object == "blue_cup":
                    judge_object_names = ["blue_cup", "blue", "cup"]
                elif command_object == "green_cup":
                    judge_object_names = ["green_cup", "green", "cup"]
                elif command_object == "orange_cup":
                    judge_object_names = ["orange_cup", "orange", "cup"]

                elif command_object == "coffee_bottle":
                    judge_object_names = ["coffee_bottle", "coffee", "bottle"]
                elif command_object == "muscat_bottle":
                    judge_object_names = ["muscat_bottle", "muscat", "bottle"]
                elif command_object == "fruits_bottle":
                    judge_object_names = ["fruits_bottle", "fruits", "bottle"]
                
                elif command_object == "penguin_doll":
                    judge_object_names = ["penguin_doll", "penguin", "doll"]
                elif command_object == "pig_doll":
                    judge_object_names = ["pig_doll", "pig", "doll"]
                elif command_object == "sheep_doll":
                    judge_object_names = ["sheep_doll", "sheep", "doll"]

                #command_object = "I"
                print("Commanded object name is " + command_object + " !")
                print("Judged object namse are " + judge_object_names[0] + " ," + judge_object_names[1] + " ," + judge_object_names[2])
                print("*********************")
                for w in object_word:
                    print(w)
                    for c in judge_object_names:
                        #print("reasoned name is " + w + " ...")
                        #print("target name is " + c + " ...")
                        if w == c:
                            print("Yes!!!!!!!")
                            result = True
                            return result
                print("*********************")
                print("************************************************************************")
        result = False
        return result

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
    #rospy.spin()