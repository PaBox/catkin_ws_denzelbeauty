#!/usr/bin/env python

import message_filters
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2 as cv

def callback(rgb_msg, camera_info):
   camera_info_K = np.array(camera_info.K).reshape([3, 3])
   camera_info_D = np.array(camera_info.D)

   #undistort and thresholding image
   rgb_image = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
   ret, rgb_image_th = cv.threshold(rgb_image, 250, 255, cv.THRESH_BINARY)

   #draw rectangles
   cv.rectangle(rgb_image_th, (230, 240), (480, 600), (0, 0, 0), -1)
   
   #resize and cut picture
   x,y,w,h = 160,50,390,280
   dst = rgb_image_th[y:y+h, x:x+w]

   rospy.loginfo("\n\n\n\n\n\n\n %s %s \n\n\n\n\n\n\n", camera_info_D,camera_info_K)
   cv.imshow("Filtered Image", dst)
   cv.waitKey(1)

if __name__ == '__main__':
   rospy.init_node('CameraInfoSub', anonymous=True)
   image_sub = message_filters.Subscriber('/sensors/camera/infra1/image_rect_raw', Image)
   info_sub = message_filters.Subscriber('/sensors/camera/infra1/camera_info', CameraInfo)
   ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.2)
   ts.registerCallback(callback)
   rospy.spin()