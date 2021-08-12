#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2

img = cv2.imread('../data/trimming/trimming_img_chair.jpg')
#cv2.imshow('color', img)
#cv2.waitKey(0)

h, w = img.shape[:2]
h1, h2 = int(h * 0.05), int(h * 0.95)
w1, w2 = int(w * 0.05), int(w * 0.95)
remove_line_img = img[h1: h2, w1: w2]
cv2.imshow('color', remove_line_img)
cv2.waitKey(0)
