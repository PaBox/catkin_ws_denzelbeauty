#!/usr/bin/env python

import message_filters
import rospy
from cv_bridge import CvBridge
from autominy_msgs.msg import SteeringAngle, Speed, Tick
from nav_msgs.msg import Odometry
import numpy as np
import cv2 as cv
import math


CV_BRIDGE = CvBridge()

INIT_FILTERED_MAP = True

FILTERED_MAP_ODOM = (0.0,0.0,0.0)

OXY_ARRAY = []

OXY_SUM = (0.0,0.0,0.0)

TICKS = 0

TIMER_UNTIL = 0

TIMER_STOP = True

def OXY_ADD(o,x,y):
    global OXY_SUM
    o_bef,x_bef,y_bef = OXY_SUM
    OXY_SUM=(o+o_bef,x+x_bef,y+y_bef)
    OXY_ARRAY.append((o,x,y))
    return OXY_SUM

def filteredmap_callback(data):
    if INIT_FILTERED_MAP:
        header = data.pose.pose
        position = header.position
        orientation = header.orientation

        global FILTERED_MAP_ODOM 
        FILTERED_MAP_ODOM = (position.x, position.y, orientation.w)

def tick_timer(data):
    global TIMER_STOP
    global TICKS
    if not TIMER_STOP:
        if(TICKS<TIMER_UNTIL):
            TICKS += data.value
        else:
            TIMER_STOP = True

def start_timer(i):
    global TIMER_UNTIL
    global TIMER_STOP
    TIMER_UNTIL = TICKS+i
    TIMER_STOP = False

def odometry_ackermann(steering_data,speed_data):
    global INIT_FILTERED_MAP
    while FILTERED_MAP_ODOM is (0.0,0.0,0.0):
        message_filters.loginfo('Searching for initial Mapping')
    if INIT_FILTERED_MAP:
        INIT_FILTERED_MAP = False
    
    if not TIMER_STOP:
        #initial values
        v = toShortFloat(speed_data.value)
        l = 0.27
        phi = steering_data.value

        tanphi = math.degrees(math.tan(phi))
        O = (v/l)*tanphi

        cosO = math.degrees(math.cos(math.radians(O)))
        X = v*cosO

        sinO = math.degrees(math.sin(math.radians(O)))
        Y = v*sinO

        OXY_ADD(O,X,Y)

    rospy.Rate(100).sleep()

def toShortFloat(f):
    return float('%.2f' % (f))

if __name__ == '__main__': 
    rospy.init_node('odometry', anonymous=True)
    rospy.loginfo('Starting...')

    initial_pos_sub = rospy.Subscriber('/sensors/localization/filteredmap', Odometry, filteredmap_callback, queue_size=10)
    initial_pos_sub = rospy.Subscriber('/sensors/arduino/ticks', Tick, tick_timer, queue_size=10)

    steering_sub = message_filters.Subscriber('/sensors/steering', SteeringAngle)
    speed_sub = message_filters.Subscriber('/sensors/speed', Speed)
    ts = message_filters.ApproximateTimeSynchronizer([steering_sub, speed_sub], 10, 0.1)
    ts.registerCallback(odometry_ackermann)

    start_timer(1000)
    rospy.sleep(3)
    rospy.loginfo(OXY_SUM)

    cv.destroyAllWindows()