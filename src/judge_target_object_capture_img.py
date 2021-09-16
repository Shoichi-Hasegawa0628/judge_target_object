#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ロボットがその場で回転しながらRGB画像を8枚保存するコード
import rospy
import math
import time
import cv2
from subprocess import * 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty
import judge_target_object_send_img


class CaptureImageCamera():
    
    def __init__(self):
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=10)
        self.vel = Twist()
        self.cv_bridge = CvBridge()
        self.image = 0
        self.taking_image()
    

    def taking_image(self):
        rospy.wait_for_message("/next_judge", Empty, timeout=None)
        time.sleep(10)
        rotations = 13
        for rotation in range(rotations):

            # π/6 (rad/s)で回転
            self.vel.angular.z = math.pi / 6
            self.vel_pub.publish(self.vel)
            time.sleep(1.0)
            self.vel.angular.z = 0
            self.vel_pub.publish(self.vel)
            time.sleep(1.0)
            print("The number of Rotation: ", rotation)

            # RGB画像を保存
            if not rotation == 12:
                object_image = self.rgb_image_ros_to_opencv()
                cv2.imwrite('../data/observation/object_image_{}.jpg'.format(rotation), object_image)

        time.sleep(1.0)
        #self.kill_node('take_observed_image')
        judge_target_object_send_img.SendObjectImage()

    def image_callback(self, img):
        self.image = img


    def rgb_image_ros_to_opencv(self):
        try:
            object_image = self.image
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
    rospy.init_node('take_observed_image')
    CaptureImageCamera()
