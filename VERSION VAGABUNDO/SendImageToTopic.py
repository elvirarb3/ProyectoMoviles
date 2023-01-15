#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

def talker(camara):

    pub = rospy.Publisher('/detect_image_input', Image, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():

        (grabbed, cv_image) = camara.read()
        cv2.imshow("Prueba", cv_image)
        k = cv2.waitKey(33)

        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(cv_image,"bgr8")
        
        rospy.loginfo("Enviando imagen")
        pub.publish(image_message)
        rate.sleep()

if __name__ == '__main__':
    
    camara = cv2.VideoCapture(0, cv2.CAP_V4L)
    try:

        talker(camara)
    except rospy.ROSInterruptException:
        pass
