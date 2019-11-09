#!/usr/bin/env python

import message_filters
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2 as cv

def callback(rgb_msg, camera_info):
   rgb_image = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
   #rgb_image_resized = cv.resize(rgb_image, None, fx=0.5, fy=0.5)
   #rgb_image_gray = cv.cvtColor(rgb_image, cv.COLOR_BGR2GRAY)
   ret, rgb_image_cvt_color = cv.threshold(rgb_image, 250, 255, cv.THRESH_BINARY)
   camera_info_K = np.array(camera_info.K).reshape([3, 3])
   camera_info_D = np.array(camera_info.D)
   #draw_img = cv.cvtColor(rgb_image_cvt_color, cv.COLOR_GRAY2RGB)
   rgb_undist = cv.undistort(rgb_image_cvt_color , camera_info_K, camera_info_D)

   rospy.loginfo("\n\n\n\n\n\n\n %s %s \n\n\n\n\n\n\n", camera_info_D,camera_info_K)
   cv.imshow("Filtered Image", rgb_undist)
   cv.waitKey(1)

if __name__ == '__main__':
   rospy.init_node('CameraInfoSub', anonymous=True)
   image_sub = message_filters.Subscriber('/sensors/camera/infra1/image_rect_raw', Image)
   info_sub = message_filters.Subscriber('/sensors/camera/infra1/camera_info', CameraInfo)
   ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.2)
   ts.registerCallback(callback)
   rospy.spin()