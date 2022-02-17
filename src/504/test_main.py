#!/usr/bin/env python

import os
import numpy as np
import cv2
import traceback
import time

from test2 import LaneDetection
from test1 import PreProcessImg

import rospy
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def img_callback(img, seq):

    start = time.time()


    detector = LaneDetection([img.copy(), seq])
    pre_process = PreProcessImg([img.copy(), seq])

    
    processedImg = pre_process.process_image()
    
    blended_img = detector.process_image(processedImg)
    
    
    cv2.imshow("processedImg", processedImg)
    cv2.imshow("blended_img", blended_img)
    
    cv2.waitKey(1)

    t = time.time() - start
    f = 1/t
    print("time : {0}, frequency: {1}".format(t, f))



class deneme():

    def __init__(self):

        rospy.Publisher("/frame", Image, queue_size=10)

        rospy.Subscriber("/frame", Image, queue_size=10)
        rospy.Subscriber("/pre", Image, queue_size=10)
        rospy.Subscriber("/out", Image, queue_size=10)

        folder = "/home/oguz/Desktop/inputs/"
    
        for filename in os.listdir(folder):
            img = cv2.imread(os.path.join(folder,filename))

            if img is not None:
                
                
                img_callback(img, filename)
                cv2.waitKey(1)
            

        



    def cam_cb(self, img):
        pass
    def pre_cb(self, img):
        pass



if __name__ == "__main__":

    rospy.init_node('test')

    

        
    rospy.spin()
    

