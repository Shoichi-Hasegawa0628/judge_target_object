#!/usr/bin/env python
# -*- coding: utf-8 -*-
# judge_yolov3.pyとjudge_mlda.pyに画像を1枚ずつ送るコード

import rospy
from sensor_msgs.msg import Image


class SendObjectImage():

    def __init__(self):
        self.img_pub = rospy.Publisher('/object_image', Image, queue_size=10)
        pass

if __name__ == "__main__":
    rospy.init_node('send_object_image')
    SendObjectImage()