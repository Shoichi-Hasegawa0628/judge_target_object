#!/usr/bin/env python
# -*- coding: utf-8 -*-
# MLDAのコードを統括するコード

import rospy
#from mlda_ros.srv import SendImageMLDA
#from mlda_ros.srv import SendImageMLDAResponse
import mlda_ros_data_image
import mlda_ros_learn
import mlda_ros_data_word


class MLDAMain():
    
    def __init__(self):
        self.mlda_image = mlda_ros_data_image.GetImageFeature()
        self.mlda_learn = mlda_ros_learn.MLDA()
        self.mlda_word = mlda_ros_data_word.GetWordFeature()
        #rospy.Service('judge_mlda', SendImageMLDA, self.judge_target_object_mlda)
    

    def judge_target_object_mlda(self, yolov3_image, status, observed_img_idx, count):
        success = 0
        if status == "estimate":
            #print("Start")                                                        # 物体の画像から物体の単語を予測
            result = self.mlda_image.image_server(yolov3_image, status, observed_img_idx, count)
            if result == 0:
                success = False
                object_word = None
                object_prob = None
                return success, object_word, object_prob
            self.mlda_learn.mlda_server(status, observed_img_idx, count)
            result = self.mlda_word.word_server(yolov3_image, status, observed_img_idx, count)
 
        #else:                                                                               # データセットからパラメータを学習
        #    pass
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