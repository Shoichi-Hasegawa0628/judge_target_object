#! /usr/bin/env python
# -*- coding: utf-8 -*-
# MLDAで使用する画像特徴量 (BoF)を計算するコード

from __init__ import *
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#from mlda_ros.srv import *
import glob
import pickle
import pathlib
import os

class GetImageFeature():
    def __init__(self):
        self.knn = cv2.ml.KNearest_create()
        self.detector = cv2.AKAZE_create()
        # self.detector = cv2.KAZE_create()
        #self.image_counter = 0
        rospy.loginfo("Ready mlda_image_server...")
        
    
    def calc_feature(self, img):
        #img = cv2.imread(filename, 0)
        kp, discriptor = self.detector.detectAndCompute(img, None)
        #extraceted_img = cv2.drawKeypoints(img, kp, None, flags=4)
        # cv2.imwrite(filename.replace('.jpg', '') + "ex.jpg",extraceted_img)
        #print(type(discriptor))
        #print(len(discriptor))
        if discriptor is None:
            print("特徴量抽出に失敗")
            return 0, 0
        elif len(discriptor) < 50:
            print("指定した特徴数よりも少ないため失敗")
            return 0, np.array(discriptor, dtype=np.float32)

        return 1, np.array(discriptor, dtype=np.float32)


    def make_codebook(self, images, code_book_size, save_name, count):
        bow_trainer = cv2.BOWKMeansTrainer(code_book_size)

        #for img in images:
        #    f = self.calc_feature(img)
        #    bow_trainer.add(f)

        result, f = self.calc_feature(images)
        if result == 0:
            return 0
        #print(f)
        bow_trainer.add(f)

        #print("Start")
        code_book = bow_trainer.cluster()
        #print("Start")

        if os.path.exists("./param/bof/{}".format(count)) is True:
            pass

        else:
            os.mkdir("./param/bof/{}".format(count))

        file = pathlib.Path("./param/bof/" + save_name)
        file.touch()
        np.savetxt("./param/bof/" + save_name, code_book)                                      
    

    def make_bof(self, code_book_name, images, hist_name, estimate_mode, count):
        code_book = np.loadtxt("./param/bof" + "/" + code_book_name, dtype=np.float32)                
        self.knn.train(code_book, cv2.ml.ROW_SAMPLE, np.arange(len(code_book), dtype=np.float32))

        hists = []
        counter = 0
        h = np.zeros(len(code_book))


        if estimate_mode:
            result, f = self.calc_feature(images)
            idx = self.knn.findNearest( f, 1 )[1]

            h = np.zeros( len(code_book) )
            for i in idx:
                h[int(i)] += 1

            hists.append( h )
            #print(hists)

            if os.path.exists("./param/bof/{}".format(count)) is True:
                pass

            else:
                os.mkdir("./param/bof/{}".format(count))

            file = pathlib.Path(hist_name)
            file.touch()

            np.savetxt( hist_name, hists, fmt=str("%d")  )
        
        """
        else:
            for img in images:
                f = self.calc_feature(img)
                idx = self.knn.findNearest(f, 1)[1]
                
                for i in idx:
                    h[int(i)] += 1

                counter += 1
                
                if counter == IMAGE_NUM:            # counterがIMAGE_NUM (4枚(1物体につき))と等しいとき
                    hists.append(h)                 # histsの配列に、
                    h = np.zeros(len(code_book))
                    counter = 0
        """
            

    def image_server(self, yolov3_image, status, observed_img_idx, count):                                  
        if status == "learn":
            pass

        elif status == "estimate":
            img = self.image_callback(yolov3_image)
            result = self.make_codebook(img, 50, "{}/codebook_{}.txt".format(observed_img_idx, count), observed_img_idx)
            if result == 0:
                return 0                                        
            self.make_bof("{}/codebook_{}.txt".format(observed_img_idx, count), img, "./param/bof/{}/histgram_v_{}.txt".format(observed_img_idx, count), True, observed_img_idx)                         


    def image_callback(self, image):
        bridge = CvBridge()
        try:
            img = bridge.imgmsg_to_cv2(image, 'passthrough')
        except CvBridgeError as e:
            print (e)
        return img
    
######################################################################################################

if __name__ == "__main__":
    rospy.init_node('mlda_image_server')
    GetImageFeature()
    #rospy.spin()
