#! /usr/bin/env python
# -*- coding: utf-8 -*-


import argparse
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image, CameraInfo


def CameraCapture(frame):
    #Puedes hace rfoto pulsando el espacio o acabar el programa pulsando la q
    bridge =CvBridge()
    cv_image =bridge.imgmsg_to_cv2(frame , "bgr8")    
    #Capturamos un frame de la cámara 
    cv2.imshow('Captura del Robot para procesar image_raw',cv_image) 
    #CAPTAMOS EL TECLADO
    k = cv2.waitKey(33)

        

if __name__ == '__main__':

    rospy.init_node('Capture_Image', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw", Image ,CameraCapture)
   #rospy.Subscriber('/detect_image_input', Image ,CameraCapture)
    
    print('Camera conectada')
    rospy.spin()
