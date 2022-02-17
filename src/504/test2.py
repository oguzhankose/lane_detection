#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 28 06:24:33 2017

@author: oguzhankose
"""
import numpy as np
import cv2
import os
import sys
import math
import time

import traceback

import matplotlib.pyplot as plt 


class LaneDetection():

    def __init__(self, img):
        
        # img[0] for img img[1] for img name
        self.orig_img = img[0]
        self.img_name = img[1]

        self.tf_img = np.zeros((250, 314), dtype="uint8")

        self.imshape = img[0].shape

        self.st_point = 200
        self.md_point = 125
        self.fi_point = 50
       

        
    def hough_lines(self, img):

        rho = 2 # distance resolution in pixels of the Hough grid
        theta = np.pi / 180 # angular resolution in radians of the Hough grid
        threshold = 20     # minimum number of votes (intersections in Hough grid cell)
        min_line_len = 20  #40 minimum number of pixels making up a line
        max_line_gap = 50    #100 maximum gap in pixels between connectable line segments
        lines = cv2.HoughLinesP(img.copy(), rho, theta, threshold, np.array([]), 
                                    minLineLength=min_line_len, maxLineGap=max_line_gap)

        return lines



    
    def tf_image(self, img, mode):

        src = np.array([[355, 204], [253, 205], [34, 335], [522, 326]], dtype=np.float32)
        dst = np.array([[70+122,0], [0+122, 0], [0+122,200], [70+122,200]], dtype=np.float32)

        if(mode == 'birdeye'):
            persp_tf = cv2.getPerspectiveTransform(src, dst)

        elif(mode == 'reverse'):
            persp_tf = cv2.getPerspectiveTransform(dst, src)

        else:
            print('Wrong Image Transform Mode')
            return False


        img = cv2.warpPerspective(img, persp_tf, (314, 250))       ### kalibrasyonndegerleri degistir class dan

        return img



    def process_image(self, pr_img):

        
        lane_found = False

        try:
            
            tf_laneContours = self.tf_image(img=pr_img.copy(), mode='birdeye')

            try:
                contours, _ = cv2.findContours(tf_laneContours.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            except:
                _, contours, _ = cv2.findContours(tf_laneContours.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            ccc = cv2.drawContours(cv2.cvtColor(tf_laneContours.copy(), cv2.COLOR_GRAY2BGR), contours, -1, (255,0,0), 3) 


        
        except Exception as e:
            print("Could NOT find any LANE")
            print(traceback.format_exc())
            lane_found = False

        

     

        return ccc

        

                
        
