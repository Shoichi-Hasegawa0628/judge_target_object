#!/usr/bin/env python
# -*- coding: utf-8 -*-
# MLDAに処理対象の画像を送り、推定結果を返すプログラム

# Standard Library
import sys

# Third Party
import rospy
import roslib.packages

# Self-made Modules
#from mlda_ros.srv import SendImageMLDA
#from mlda_ros.srv import SendImageMLDAResponse
sys.path.append(str(roslib.packages.get_pkg_dir("mlda")) + "/scripts")
import extract_img_bof
import execute_node
import extract_word_bow


class MLDAMain():
    
    def __init__(self):
        self.mlda_image = extract_img_bof.ExtractImgBof()
        self.mlda_learn = execute_node.MLDA()
        self.mlda_word = extract_word_bow.ExtractWordBow()
        #rospy.Service('judge_mlda', SendImageMLDA, self.judge_target_object_mlda)
    

    def judge_target_object_mlda(self, yolov3_image, status, observed_img_idx, count):
        if status == "estimate":
            result = self.mlda_image.image_server(status, count, observed_img_idx, yolov3_image)
            if result == 0:
                success = False
                object_word = None
                object_prob = None
                return success, object_word, object_prob
            self.mlda_learn.mlda_server(status, observed_img_idx, count)
            result = self.mlda_word.word_server(yolov3_image, status, observed_img_idx, count)

        object_word = []
        #object_word_list = []
        object_prob = []
        #object_prob_list = []

        #print(result)
        for w in result.keys():
            key_start = w.find('_')
            #key_goal = len(w)
            object_word.append(w[key_start+1 :])
            #print(object_word)
        #object_word_list.append(object_word)

        for p in result.values():
            object_prob.append(p)
        
        #print ("Finished!!!")
        success = True
        #return SendImageMLDAResponse(success = True, object_word = "apple", object_word_probability = 0.1)
        return success, object_word, object_prob


if __name__ == "__main__":
    rospy.init_node('mlda_main_server')
    MLDAMain()
    rospy.spin()