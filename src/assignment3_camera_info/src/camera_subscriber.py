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

   #define sections for left and right side
   pleft1 = dst[0:90, 0:195]
   pleft2 = dst[90:150, 0:195]
   pleft3 = dst[150:280, 0:195]

   pright1 = dst[0:80, 195:390]
   pright2 = dst[80:130, 195:390]
   pright3 = dst[130:280, 195:390]

   #define array of image parts
   img_arr = [pleft1,pleft2,pleft3,pright1,pright2,pright3]

   #for loop to cycle through image parts
   img_index=0
   for img in img_arr:
      whitePointList = np.argwhere( np.asarray(img) >= 245 )
      rospy.loginfo(whitePointList)
      numWhitePoints = len( whitePointList )
      setwpoint = [0,0]
      for wpoint in whitePointList:
         setwpoint[0]+=wpoint[0]
         setwpoint[1]+=wpoint[1]

      setwpoint = [
         abs(setwpoint[0]/numWhitePoints),
         abs(setwpoint[1]/numWhitePoints)
      ]

      newkp = cv.KeyPoint(setwpoint[0],setwpoint[1],1)

      pkey = cv.drawKeypoints(img, [newkp], np.array([]), (0,0,255))
      
      img_index = img_index + 1
      img_title = 'Image Part Nr.{}'.format(img_index)
      cv.imshow(img_title, pkey)

   rospy.loginfo("\n\n\n\n\n\n\n %s %s \n\n\n\n\n\n\n", camera_info_D,camera_info_K)
   cv.waitKey(1)

if __name__ == '__main__':
   rospy.init_node('CameraInfoSub', anonymous=True)
   image_sub = message_filters.Subscriber('/sensors/camera/infra1/image_rect_raw', Image)
   info_sub = message_filters.Subscriber('/sensors/camera/infra1/camera_info', CameraInfo)
   ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.2)
   ts.registerCallback(callback)
   rospy.spin()