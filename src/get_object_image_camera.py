#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ロボットがその場で回転しながらRGB画像を8枚保存し, YOLOv3にRGB画像を配信するコード
import rospy
import math
import time
import cv2
from subprocess import * 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class GetObjectImageCamera():
    
    def __init__(self):
        self.img_pub = rospy.Publisher('/object_image', Image, queue_size=10)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=10)
        self.vel = Twist()
        self.cv_bridge = CvBridge()
        self.image = 0
        self.taking_image()
    

    def taking_image(self):
        rotations = 9
        for rotation in range(rotations):

            # π/4 (rad/s)で回転
            self.vel.angular.z = math.pi / 4
            self.vel_pub.publish(self.vel)
            time.sleep(1.0)
            self.vel.angular.z = 0
            self.vel_pub.publish(self.vel)
            time.sleep(1.0)
            print("The number of Rotation: ", rotation)

            # RGB画像をデバック用に保存し, YOLOv3へ配信する
            if not rotation == 8:
                object_image = self.rgb_image_ros_to_opencv()
                cv2.imwrite('./data/object_image_{}.jpg'.format(rotation), object_image)

        time.sleep(1.0)
        self.kill_node('get_object_image_yolov3')

    def image_callback(self, img):
        self.image = img


    def rgb_image_ros_to_opencv(self):
        try:
            object_image = self.image
            print(type(object_image))
            self.img_pub.publish(object_image)
            object_image = self.cv_bridge.imgmsg_to_cv2(object_image, 'passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)

        object_image = cv2.cvtColor(object_image, cv2.COLOR_BGR2RGB)
        return object_image


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
    rospy.init_node('get_object_image_yolov3')
    GetObjectImageCamera()
