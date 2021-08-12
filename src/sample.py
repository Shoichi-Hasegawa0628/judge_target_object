#!/usr/bin/env python
# -*- coding: utf-8 -*-
# YOLOv3で検出した物体から, 識別精度0.7以上の物体のみtarget_objectの候補として登録するコード
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from sensor_msgs.msg import Image

im = cv2.imread('../data/trimming/trimming_img_tvmonitor.jpg')
h, w = im[:2]
h1, h2 = int(h * 0.05), int(h * 0.95)
w1, w2 = int(w * 0.05), int(w * 0.95)
remove_line_img = im[h1: h2, w1: w2]
cv2.imshow('color', remove_line_img)
cv2.waitKey(0)
#print(im)