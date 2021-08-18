#! /usr/bin/env python
# -*- coding: utf-8 -*-
# MLDAで使用する画像特徴量 (BoF)を計算するコード

"""
import cv2
origin = cv2.imread("../data/observation/object_image_0.jpg")
height_o = origin.shape[0]
width_o = origin.shape[1]

img = cv2.imread("../data/trimming/0/trimming_img_0.jpg")
height = img.shape[0]
width = img.shape[1]

#img3 = cv2.resize(img , (int(width*1.5), int(height*1.5)))
img3 = cv2.resize(img , (int(width_o), int(height_o )))
cv2.imwrite('../data/trimming/0/trimming_img_0_re.jpg' , img3)
"""
import pathlib
import numpy as np

save_name = "{}/codebook_{}.txt".format(0, 0)
file = pathlib.Path("../data/bof/" + save_name)
file.touch()
code_book = np.zeros(10)
np.savetxt("./data/bof/" + save_name, code_book)  