#!/usr/bin/env python

import message_filters
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2 as cv


CV_BRIDGE = CvBridge()

CV_TRESHOLD = 200

BLACK_RECTANGLES = [
   [(0, 0), (640, 160)], #header
   [(550, 160), (640, 480)], #right side
   [(460, 160), (550, 360)], #right side
   [(390, 160), (460, 240)], #right side
   [(330, 160), (390, 180)], #right side
   [(195, 320), (395, 400)], #car
   [(160, 400), (460, 480)]  #car
]

FOCUS_AREA = [
   [0,160],[100,560]
]

RANSAC_MASKS = [ # FOCUS_AREA_FORMAT
   [(0,0),(200,100)],
   [(200,0),(300,100)],
   [(300,0),(400,100)]
]


def callback(image):

   #undistort and thresholding image
   infra_image = CV_BRIDGE.imgmsg_to_cv2(image, desired_encoding="bgr8")
   infra_image_th = blackoutImage(infra_image)
   infra_image_th_rf = cutRansacMasks(infra_image_th)

   #define sections for left and right side
   #cv.imshow("Original", rgb_image)
   #cv.imshow("Cam", infra_image_th_rf)
   cv.imshow("Cam uncut", infra_image_th)

   rospy.loginfo(infra_image_th)
   cv.waitKey(1)

def functionresult(x, m, b):
   return m*x+b

def returnLists(image):
   return (image[:,1], image[:,0])

def testVarCov(points):
   (x,y) = returnLists(points)
   (xm,ym) = np.mean(x),np.mean(y)

   xdiff = (x-xm)
   var = np.sum(xdiff *  (y-ym))
   cov = np.sum(xdiff ** 2)

   out_m = ( var / cov )
   out_b = (ym - (xm*out_m))

   return (out_m, out_b)

def ransac(candidates, itr, thr, perc):
   best_fitting = None
   max_fitting_score = 0
   min_coeff = 20

   for _ in range(itr):
      evolution = np.random.permutation(candidates)[:min_coeff]
      m, b = testVarCov(evolution)
      out = notFitting(candidates, m, b)
      fitting_all = candidates[out<thr]
      fitting_score = len(fitting_all)

      if len(candidates) != 0:
         if (fitting_score / float(len(candidates)) < perc) or (fitting_score<min_coeff): continue
      if (fitting_score > max_fitting_score):
           max_fitting_score = fitting_score
           m, b = testVarCov(fitting_all)

   if max_fitting_score == None:
      rospy.loginfo("NO FITS FOUND")
      raise Exception("NO FITS")
   else:
      return (m,b)
           



def notFitting(image, m, b):
   (x,y) = returnLists(image)
   result = functionresult(x,m,b)
   return (y - result)**2

   (x,y) = image[1], image[0]
   result = functionresult(x,m,b)
   return (y - result)**2

def cutRansacMasks(image):
   cut_image = image[FOCUS_AREA[0][1]:FOCUS_AREA[1][1],FOCUS_AREA[0][0]:FOCUS_AREA[1][0]]
   for ((x1,y1),(x2,y2)) in RANSAC_MASKS:
      c_i_p = cut_image[y1:y2,x1:x2]
      cv.imshow("DDS", cut_image)
      whitePointList = np.argwhere( c_i_p )
      m, b = ransac(whitePointList,5,CV_TRESHOLD,5)

      (x1,x2) = np.min(whitePointList[:,1]), np.max(whitePointList[:,1])
      (y1,y2) = (x1*m) + b, (x2*m) + b
      cv.line(cut_image, (int(x1),int(y1)), (int(x2),int(y2)),255,thickness=5)
   return cut_image

def blackoutImage(image):
   #make optimal thresholded image
   _, infra_image_th = cv.threshold(image, CV_TRESHOLD, 255, cv.THRESH_BINARY)

   #black out sections efficently
   for ((x1,y1),(x2,y2)) in BLACK_RECTANGLES:
      infra_image_th[y1:y2,x1:x2] = 0

   return infra_image_th

   

if __name__ == '__main__':
   rospy.init_node('CameraInfoSub', anonymous=True)

   image_sub = rospy.Subscriber('/sensors/camera/infra1/image_rect_raw', Image, callback, queue_size=10)
   image_pub = rospy.Publisher('/sensors/camera/infra1/line_detection', Image, queue_size=10)

   rospy.spin()