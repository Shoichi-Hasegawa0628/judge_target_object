#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import numpy as np
import random
import subprocess
import math

#from mlda_ros.srv import *
#from mlda_ros.msg import *

import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import roslib.packages


ITERATION = 100
CATEGORYNUM = 3
#CATEGORYEXAMPLENUM = 3
#IMAGE_NUM = 4
ALPHA = 1.0
BETA = 1.0

JUDGE_TARGET_OBJCT_PATH = str(roslib.packages.get_pkg_dir("judge_target_object"))
OBSERVATION_FOLDER = JUDGE_TARGET_OBJCT_PATH  + "/data/observation/"
RESIZE_FOLDER = JUDGE_TARGET_OBJCT_PATH  + "/data/resize/"
TRIMMING_FOLDER = JUDGE_TARGET_OBJCT_PATH  + "/data/trimming/"
YOLO_IMG_FOLDER = JUDGE_TARGET_OBJCT_PATH + "/data/yolov3/"