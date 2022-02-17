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
import matplotlib.pyplot as plt


class PreProcessImg():

    def __init__(self, img):
        

        #   ###
        #   1 Take Image and Seq as Input
        # img[0] for img img[1] for img name
        self.orig_img = img[0]
        self.img_name = img[1]
        self.imshape = img[0].shape


        #cv2.imshow("1 orig_img", self.orig_img)
        #cv2.waitKey(0)



    def process_image(self):

        #   ###
        #   2 Grayscale one color channel
        grayImg = cv2.cvtColor(self.orig_img.copy(), cv2.COLOR_BGR2GRAY)


        #   ###
        #   3 Gaussian Blur
        """
        Use low pass filter to remove noise. Will remove high freq stuff like noise and edges
        kernel_size specifies width/height of kernel, should be positive and odd
        Also specify stand dev in X and Y direction, give zero to calculate from kernel size
        Can also use average, median, and bilarteral blurring techniques
        """
        
        smoothed = cv2.bilateralFilter(grayImg.copy(), 11, 75, 75)


        #   ###
        #   4 Create mask to only keep area defined by four coners
        # Black out every area outside area
        vertices = np.array([[(0,self.imshape[0]), (0, 210), (self.imshape[1], 210), (self.imshape[1],self.imshape[0])]], dtype=np.int32)
        # defining a blank mask to start with, 0s with same shape of edgesImg
        # fill pixels inside the polygon defined by vertices"with the fill color
        mask = np.zeros_like(smoothed)  
        cv2.fillPoly(mask, vertices, 255)
        # create image only where mask and edge Detection image are the same
        maskedImg = cv2.bitwise_and(smoothed.copy(), mask)



        #   ###
        #   5 Adaptive Threshold for Filtering
        """
        src – Source 8-bit single-channel image.
        dst – Destination image of the same size and the same type as src .
        maxValue – Non-zero value assigned to the pixels for which the condition is satisfied. See the details below.
        adaptiveMethod – Adaptive thresholding algorithm to use, ADAPTIVE_THRESH_MEAN_C or ADAPTIVE_THRESH_GAUSSIAN_C . See the details below.
        thresholdType – Thresholding type that must be either THRESH_BINARY or THRESH_BINARY_INV .
        blockSize – Size of a pixel neighborhood that is used to calculate a threshold value for the pixel: 3, 5, 7, and so on.
        C – Constant subtracted from the mean or weighted mean (see the details below). Normally, it is positive but may be zero or negative as well.
        """
        thresholdImg = cv2.adaptiveThreshold(maskedImg.copy(), 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                          cv2.THRESH_BINARY_INV, 85, 22)


        #  Clear the Horizon that caused by masking image
        thresholdImg[190:220] = 0
        thresholdImg[thresholdImg.shape[0]-20:thresholdImg.shape[0]] = 0
 

       

        return thresholdImg

        

        
       



        
        





        

                
        
