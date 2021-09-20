# 'judge_target_object' Package

This ROS package is to judge target object from human command by using MLDA and YOLOv3.

*   Maintainer: Shoichi Hasegawa ([hasegawa.shoichi@em.ci.ritsumei.ac.jp](mailto:hasegawa.shoichi@em.ci.ritsumei.ac.jp)).
*   Author: Shoichi Hasegawa ([hasegawa.shoichi@em.ci.ritsumei.ac.jp](mailto:hasegawa.shoichi@em.ci.ritsumei.ac.jp)).

**Content:**
*   [Requirements](#requirements)
*   [Launch](#launch)
*   [Codes](#codes)
*   [References](#references)

## Requirement
~~~
yolov3  
pip install pathlib  
pip install natsort  
~~~

## Launch
~~~
roslaunch judge_target_object judge_target_object.launch
~~~

## Codes
- `README.md`: Read me file (This file)

- `judge_target_object.launch`: Execute judge_target_object by launch file.

- `judge_target_object_capture_img.py`: Capture image with camera by robot

- `judge_target_object_send_img.py`:  Send one image to YOLOv3 and MLDA

- `judge_target_object_cut_image.py`:  Detect object in a image with yolo, cut this object image

- `judge_target_object_mlda_main.py`:  Manage mlda codes

- `judge_target_object_mlda_learn.py`:  Learn parameters by mlda

- `judge_target_object_mlda_word.py`:  Extarct word feature (BoW)

- `judge_target_object_mlda_image.py`:  Extract image feature (BoF)

## References











