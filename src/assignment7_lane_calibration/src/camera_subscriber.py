#!/usr/bin/env python

import message_filters
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2 as cv


CV_BRIDGE = CvBridge()

CV_THRESHOLD = 200

BLACK_RECTANGLES = [
   [(0, 0), (640, 160)], #header
   [(550, 160), (640, 480)], #right side
   [(460, 160), (550, 360)], #right side
   [(390, 160), (460, 240)], #right side
   [(330, 160), (390, 180)], #right side
   [(195, 320), (395, 400)], #car
   [(160, 400), (460, 480)]  #car
]

RANSAC_MASKS = [
   [(0,160),(200,260)],
   [(200,160),(300,360)],
   [(300,160),(500,480)]
]


def callback(image):

   #undistort and thresholding image
   infra_image = CV_BRIDGE.imgmsg_to_cv2(image, desired_encoding="8UC1")
   infra_image_th = blackoutImage(infra_image)
   infra_image_th_rf,bg,nl = cutRansacMasks(infra_image_th,infra_image)

   #define sections for left and right side
   cv.imshow("Original", bg)
   cv.imshow("Only Mask", nl)
   cv.imshow("Mixed Output", infra_image_th_rf)


   image_pub.publish(CV_BRIDGE.cv2_to_imgmsg(infra_image_th_rf, "8UC1"))

   cv.waitKey(1)

def functionresult(x, par):
   m,b = par
   return m*x+b

def returnLists(image):
   return image[:, 1], image[:, 0]

def testVarCov(points):
   x, y = returnLists(points)
   xm,ym = np.mean(x),np.mean(y)

   variance = np.sum((x-xm) *  (y-ym))
   covariance = np.sum((x-xm) ** 2)

   out_m = ( variance / covariance )
   out_b = ym - out_m*xm
   #gives out pars
   #rospy.loginfo('(m: "{}",b: "{}")'.format(out_m,out_b))
   return out_m, out_b

def ransac(candidates, itr, thr, perc):
   best_fitting = None
   max_fitting_score = 0
   min_coeff = 20

   for _ in range(itr):
      evolution = np.random.permutation(candidates)[:min_coeff]
      #rospy.loginfo(evolution)
      par = testVarCov(evolution)
      out = notFitting(candidates, par)
      fitting_all = candidates[out < thr]
      fitting_score = len(fitting_all)
      #rospy.loginfo(fitting_all)

      if (
         fitting_score / float(len(candidates)) < (perc/100) or fitting_score<min_coeff
      ):
         continue
      if (fitting_score > max_fitting_score):
           max_fitting_score = fitting_score
           best_fitting = testVarCov(fitting_all)
           #log succesful choice of point
           rospy.loginfo(best_fitting)

   if best_fitting is None:
      rospy.loginfo("NO FITS FOUND")
      raise Exception("NO FITS")
   else:
      return best_fitting
           
def recFor(pics, x1, y1, x2, y2, clr,th):
   for pic in pics:
      cv.rectangle(pic, (int(x1),int(y1)), (int(x2),int(y2)),clr,th)

def lineFor(pics, x1, y1, x2, y2, clr,th):
   for pic in pics:
      cv.line(pic, (int(x1),int(y1)), (int(x2),int(y2)),clr,th)


def notFitting(image, par):
   (x,y) = returnLists(image)
   result = functionresult(x,par)
   return (y - result)**2

def cutRansacMasks(image,bg):
   nulled = np.zeros_like(image)
   consider = np.argwhere(image)
   for (x1,y1),(x2,y2) in RANSAC_MASKS:
      c_i_p = np.array([(y, x) for (y, x) in consider if x1 <= x <= x2 and y1 <= y <= y2])
      m, b = ransac(c_i_p,5,CV_THRESHOLD,90)
      #scanning area
      recFor([bg], x1, y1, x2, y2, 250, 3)
      
      x1, x2 = np.min(c_i_p[:, 1]), np.max(c_i_p[:, 1])
      (y1,y2) = (x1*m) + b, (x2*m) + b
      #line function
      recFor([image, bg], x1, y1, x2, y2, 160, 1)
      lineFor([nulled, image, bg], x1, y1, x2, y2, 160, 1)
   
   return image, bg, nulled

def blackoutImage(image):
   #make optimal thresholded image
   _, infra_image_th = cv.threshold(image, 250, 255, cv.THRESH_BINARY)

   #black out sections efficently
   for ((x1,y1),(x2,y2)) in BLACK_RECTANGLES:
      infra_image_th[y1:y2,x1:x2] = 0

   return infra_image_th

   

if __name__ == '__main__':
   rospy.init_node('CameraInfoSub', anonymous=True)

   image_sub = rospy.Subscriber('/sensors/camera/infra1/image_rect_raw', Image, callback, queue_size=10)
   image_pub = rospy.Publisher('/sensors/camera/infra1/line_detection', Image, queue_size=10)

   rospy.spin()