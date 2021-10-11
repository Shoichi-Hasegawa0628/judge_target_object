#! /usr/bin/env python
# -*- coding: utf-8 -*-

# 必要なライブラリ
# pip install pathlib

from __future__ import unicode_literals
from __init__ import *
import glob
import codecs
from std_msgs.msg import String
import pathlib
import os
import rospy

class GetWordFeature():
    """
    def separate_word(self, filename):
        for line in codecs.open(DATA_FOLDER + "/" + filename, "r", "utf-8").readlines():
            line = line.rstrip("\r\n")
            words = line.split(" ")
            self.lines.append(words)
        rospy.loginfo("[Service em_mlda/data] finish to separate words")


    def make_dic(self, dictionary_name):
        for line in self.lines:
            for word in line:
                if word not in self.dictionary:
                    self.dictionary.append(word)
                    
        codecs.open(DATA_FOLDER + "/" + dictionary_name, "w", "utf-8").write("\n".join(self.dictionary))
        rospy.loginfo("[Service em_mlda/data] save dictionary as %s", dictionary_name)
        

    def make_bow(self, hist_name):
        hist = np.zeros((len(self.lines), len(self.dictionary)))
        
        for d, words in enumerate(self.lines):
            for word in words:
                idx = self.dictionary.index(word)
                hist[d, idx] += 1
                
        np.savetxt(DATA_FOLDER + "/" + hist_name, hist, fmt=str("%d"))
        rospy.loginfo("[Service em_mlda/data] save the histgram as %s", hist_name)
    

    def add_word(self,add_word ,object_id):
        with open(DATA_FOLDER + "/word.txt") as f:
            current_words = [s.strip() for s in f.readlines()]
        
        current_words[object_id] = current_words[object_id] + " " + add_word
        
        with open(DATA_FOLDER + "/word.txt", mode='w') as f:
            f.write('\n'.join(current_words))
            
        rospy.loginfo("add the new word %s to object %d", add_word, object_id)

    """
    def estimate(self, yolov3_image, status, observed_img_idx, count):
        word = np.loadtxt("./param/estimate_result/{}/Pmdw[1]_{}.txt".format(observed_img_idx, count))
        word_file = open("./param/bow/word_dic.txt")
        word_dic = word_file.readlines()
        word_dic = [convert.replace("\n", "") for convert in word_dic]
        word_dic = [convert.replace(".", "") for convert in word_dic]
        #print(word_dic)
        
        unsorted_max_indices = np.argpartition(-word, 10)[:10]                    # 上位10つの単語を表示
        #print(unsorted_max_indices)
        #print(len(unsorted_max_indices))
        #print(len(word))
        #print(word)

        y = word[unsorted_max_indices]
        #print(y)
        indices = np.argsort(-y)
        max_k_indices = unsorted_max_indices[indices]
        
        rospy.loginfo("Estimate the word as")

        index_counter = 0
        
        estimated_words = "I estimated this object. "
        estimate_result = {}
        
        with open("./param/estimate_result/{}/estimate_word_{}.txt".format(observed_img_idx, count), mode='w') as f:
            for word_index in max_k_indices:
                #print(word_dic[word_index])
                # estimated_words += word_dic[word_index] + " " + str(round(y[index_counter] * 100, 2)) + "%. "
                estimated_words += "No." + str(index_counter + 1) + ". " + word_dic[word_index] + " " + str(round(y[index_counter] * 100, 1)) + "%. " 
                # estimated_words += word_dic[word_index] + "."
                f.write(word_dic[word_index] + " " + str(round(y[index_counter] * 100, 1)) + "%" + "\n")
                estimate_result["{}{}_".format(observed_img_idx, count) + word_dic[word_index]] = round(y[index_counter] * 100, 1)
                index_counter += 1
        
        estimate_hist = np.zeros(len(word_dic))
        
        for word_index in max_k_indices:
            estimate_hist[word_index] += 1
        
        if os.path.exists("./param/estimate_result/{}".format(observed_img_idx)) is True:
            pass

        else:
            os.mkdir("./param/estimate_result/{}".format(observed_img_idx))

        file = pathlib.Path("./param/estimate_result/{}/estimate_histogram_word_{}.txt".format(observed_img_idx, count))
        file.touch()
        np.savetxt("./param/estimate_result/{}/estimate_histogram_word_{}.txt".format(observed_img_idx, count), estimate_hist.reshape(1, -1), fmt=str("%d"))

        return estimate_result
        
    
    def word_server(self, yolov3_image, status, observed_img_idx, count):
        if status == "learn":
            #self.separate_word("word.txt")
            #self.make_dic("word_dic.txt")
            #self.make_bow("histogram_word.txt")
            pass

        elif status == "estimate":
            result = self.estimate(yolov3_image, status, observed_img_idx, count)
            return result
            #return mlda_wordResponse(self.estimate_word(count))

        #else:
        #    self.add_word(sentence, count)         
        
        #return mlda_wordResponse("Finished")


    def __init__(self):
        #s = rospy.Service('em_mlda/data/word', mlda_word, self.word_server)
        rospy.loginfo("Ready em_mlda/word")
        self.lines = []
        self.dictionary = []


if __name__ == "__main__":
    rospy.init_node('em_mlda_word_server')
    GetWordFeature()
    #rospy.spin()
