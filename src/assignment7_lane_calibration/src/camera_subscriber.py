#!/usr/bin/env python

import message_filters
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2 as cv

def callback(rgb_msg):

   #undistort and thresholding image
   rgb_image = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
   ret, rgb_image_th = cv.threshold(rgb_image, 250, 255, cv.THRESH_BINARY)

   #draw rectangles
   cv.rectangle(rgb_image_th, (400, 0), (600, 600), (0, 0, 0), -1)
   cv.rectangle(rgb_image_th, (0, 0), (600, 180), (0, 0, 0), -1)
   cv.rectangle(rgb_image_th, (0, 300), (600, 600), (0, 0, 0), -1)
   
   #resize and cut picture
   x,y,w,h = 0,180,400,120
   dst = rgb_image_th[y:y+h, x:x+w]

   #define sections for left and right side
   

   #define array of image parts

   #for loop to cycle through image parts
   cv.imshow("Cam", dst)
   cv.imshow("Cam uncut", rgb_image_th)

   rospy.loginfo(rgb_image)
   cv.waitKey(1)

if __name__ == '__main__':
   rospy.init_node('CameraInfoSub', anonymous=True)

   image_sub = rospy.Subscriber('/sensors/camera/infra1/image_rect_raw', Image, callback, queue_size=10)

   rospy.spin()