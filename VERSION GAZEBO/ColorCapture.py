#! /usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int16

class DetectColors():
    def __init__(self):

        self.cvBridge = CvBridge()
        self.cv_image = None 
        self.counter = 1
        self.is_image_available = False
        self.is_solution_send = False

        #Configuramos los parametros para hacer las mascaras de los colores 
        #DETECTAMOS EL ROJO 
        self.hue_red_l = 160 #0
        self.hue_red_h = 179 #60 #26
        self.saturation_red_l =  50 #100#239
        self.saturation_red_h = 255
        self.lightness_red_l = 0 #100#123
        self.lightness_red_h = 255#250

        #AHORA EL AMARILLO ES EL AZUL 
        self.hue_yellow_l = 95 #90 depende luz
        self.hue_yellow_h = 130 #140
        self.saturation_yellow_l = 50 #100
        self.saturation_yellow_h = 255
        self.lightness_yellow_l = 0 #131
        self.lightness_yellow_h = 255

        #DETECTAMOS EL VERDE
        self.hue_green_l = 40
        self.hue_green_h = 90
        self.saturation_green_l = 50 #100
        self.saturation_green_h = 255
        self.lightness_green_l = 50 #100
        self.lightness_green_h = 255

        self.min_azul = 18000
        self.min_rojo = 20000
        self.min_verde = 10000

        #Nos subscribimos a un topic 
        self.sub_image = rospy.Subscriber('/detect_image_input', Image, self.GetImage, queue_size = 1)
        rospy.sleep(1)


        self.is_solution_send =False
        #Publicamos en este topic --> self.pub_color_detection.publish(msg_sign)
        self.pub_color_detection = rospy.Publisher('Color_detection', Image, queue_size = 1)
        
        loop_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_image_available == True:
                if self.is_solution_send== False: #ESTA LINEA Y VARAIBLE NO VALE PARA NADA 
                    self.FindColor()
            loop_rate.sleep()

    def FindColor(self):
        cv_image_mask = self.fnMaskGreenTrafficLight()
        cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)
   
        cv_image_mask = self.fnMaskYellowTrafficLight()
        cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)
        
        cv_image_mask = self.fnMaskRedTrafficLight()
        cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)


    def fnMaskRedTrafficLight(self):
        image = np.copy(self.cv_image)
        
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #cv2.imshow('ELRojohsv',hsv) 

        Hue_l = self.hue_red_l
        Hue_h = self.hue_red_h
        Saturation_l = self.saturation_red_l
        Saturation_h = self.saturation_red_h
        Lightness_l = self.lightness_red_l
        Lightness_h = self.lightness_red_h

        # define range of red color in HSV
        lower_red = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_red = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)
        #cv2.imshow('LaMascaraRoja',mask)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)
        cv2.imshow('ROJO',res) 
        col = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)
        gray = cv2.cvtColor(col, cv2.COLOR_BGR2GRAY)
        contours, hi = cv2.findContours(gray, 1,2) #cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        min_area = 20000
        detectado = False
        if len(contours) != 0:
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.min_rojo:
                    print ("Rojo puñetazo en el ojo")
                    detectado = True
        
        if detectado:
            pub_rojo.publish(1)
        else:
            pub_rojo.publish(0)

        k = cv2.waitKey(33)
        # publishes red light filtered image in raw type
        
        return mask

    def fnMaskYellowTrafficLight(self):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_yellow_l
        Hue_h = self.hue_yellow_h
        Saturation_l = self.saturation_yellow_l
        Saturation_h = self.saturation_yellow_h
        Lightness_l = self.lightness_yellow_l
        Lightness_h = self.lightness_yellow_h

        # define range of yellow color in HSV
        lower_yellow = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_yellow = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)
        cv2.imshow('Azul',res)
        col = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)
        gray = cv2.cvtColor(col, cv2.COLOR_BGR2GRAY)
        contours, hi = cv2.findContours(gray, 1,2) #cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        min_area = 7000
        detectado = False
        if len(contours) != 0:
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.min_azul:
                    print ("Azul como el mar")
                    detectado = True

        if detectado:
            pub_azul.publish(1)
        else:
            pub_azul.publish(0)
        
        return mask

    def fnMaskGreenTrafficLight(self):
       
        image = np.copy(self.cv_image)


        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_green_l
        Hue_h = self.hue_green_h
        Saturation_l = self.saturation_green_l
        Saturation_h = self.saturation_green_h
        Lightness_l = self.lightness_green_l
        Lightness_h = self.lightness_green_h

        # define range of green color in HSV
        lower_green = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_green = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)
        cv2.imshow('Verde',res) 
        col = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)
        gray = cv2.cvtColor(col, cv2.COLOR_BGR2GRAY)
        contours, hi = cv2.findContours(gray, 1,2) #cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        min_area = 1000
        detectado = False
        if len(contours) != 0:
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.min_verde:
                    print ("Verde que te quiero verde")
                    detectado = True
        
        if detectado:
            pub_verde.publish(1)
        else:
            CameraCapture
            pub_verde.publish(0)
            
        return mask   
   
    def GetImage(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        self.is_image_available = True


def CameraCapture(frame):
    #Puedes hace rfoto pulsando el espacio o acabar el programa pulsando la q
    bridge =CvBridge()
    cv_image =bridge.imgmsg_to_cv2(frame , "bgr8")    
    #Capturamos un frame de la cámara 
    cv2.imshow('Captura del Robot para procesar image_raw',cv_image) 
    #CAPTAMOS EL TECLADO
    k = cv2.waitKey(33)


if __name__ == '__main__':
    
    pub_rojo = rospy.Publisher('/detect/rojo', Int16, queue_size=5)
    pub_verde = rospy.Publisher('/detect/verde', Int16, queue_size=5)
    pub_azul = rospy.Publisher('/detect/azul', Int16, queue_size=5)
    rospy.init_node('detect_color')
    node = DetectColors()
    node.main()
