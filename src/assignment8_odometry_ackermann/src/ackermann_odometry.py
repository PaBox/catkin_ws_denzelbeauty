#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from autominy_msgs.msg import SteeringAngle, Speed
from nav_msgs.msg import Odometry
import numpy as np
import cv2 as cv
import math


CV_BRIDGE = CvBridge()
RP_RATE = 100
CAR_LENGTH = 0.27

INIT_FILTERED_MAP = False
GPS_ODOM = None

SPEED = 0.0
STEER = 0.0

OXY_SUM = (0.0,0.0,0.0,0.0)
OXY_ODOM = Odometry()

TIME_LAST = None
SHORTOUTFLOATS = False

def toShortFloat(f):
    if SHORTOUTFLOATS:
        return float('%.3f' % (f))
    return f

def getTime():
    global TIME_LAST
    TIME_NOW = rospy.Time.now()
    TIME_LAST = TIME_NOW
    return TIME_NOW

def getTimeDelta():
    global TIME_LAST
    ts = TIME_LAST
    return (getTime() - ts).to_sec()

def OXYTODOM(o,x,y,z,t):
    global OXY_ODOM

    #edit own odom
    header = OXY_ODOM.pose.pose
    position = header.position
    orientation = header.orientation

    half_o = o / 2

    orientation.w = math.cos(half_o)
    orientation.z = math.sin(half_o)
    position.x = x
    position.y = y
    position.z = z

    rospy.loginfo(OXY_ODOM)
    denzel_odom_pub.publish(OXY_ODOM)

def speed_callback(data):
    global SPEED
    SPEED = data.value

def steer_callback(data):
    global STEER
    STEER = data.value

def gps_callback(data):
    global GPS_ODOM
    GPS_ODOM = data

def filteredmap_callback(data):
    #INITIALISING VALUES FROM FILTERED MAP
    global INIT_FILTERED_MAP, OXY_SUM
    if not INIT_FILTERED_MAP:
        #setup paths for filtered map odom
        headerFM = data.pose.pose
        positionFM = headerFM.position
        orientationFM = headerFM.orientation

        #setup one time variables for own odom
        OXY_SUM = (math.acos(orientationFM.w), positionFM.x, positionFM.y, positionFM.z)
        rospy.loginfo(OXY_SUM)
        INIT_FILTERED_MAP = True

def odometry_ackermann(timestamp, speedstamp, steerstamp):
    global OXY_SUM

    #initial values
    v = toShortFloat(speedstamp)
    l = CAR_LENGTH
    phi = toShortFloat(steerstamp)
    o_bef,x_bef,y_bef, z = OXY_SUM

    O = (v/l)*math.tan(phi)
    X = v*math.cos(o_bef)
    Y = v*math.sin(o_bef)

    delta = toShortFloat(timestamp)
    
    #calc summed values of odom
    o_aft = o_bef + (delta * O)
    x_aft = x_bef + (delta * X)
    y_aft = y_bef + (delta * Y)

    OXY_SUM=(o_aft,x_aft,y_aft,z)

    OXYTODOM(o_aft,x_aft,y_aft,z,timestamp)

    rospy.Rate(100).sleep()

if __name__ == '__main__': 
    rospy.init_node('denzel_odometry', anonymous=True)
    rospy.loginfo('Starting...')

    #Publisher for own Odometry
    denzel_odom_pub = rospy.Publisher('denzel_odom', Odometry, queue_size=10)

    #Subscribers for internal ticks and filteredmap for inital position
    initial_pos_sub = rospy.Subscriber('/sensors/localization/filtered_map', Odometry, filteredmap_callback, queue_size=10)
    gps_sub = rospy.Subscriber('/communication/gps/16', Odometry, gps_callback, queue_size=10)

    #subscribers for syncing callback - improved with asynchrous callback and global values
    steer_sub = rospy.Subscriber('/sensors/steering', SteeringAngle, steer_callback, queue_size=10)
    speed_sub = rospy.Subscriber('/sensors/speed', Speed, speed_callback, queue_size=10)

    #set rate of odometry beeing called
    rate = rospy.Rate(RP_RATE)

    #creating own odom infos
    OXY_ODOM.header.frame_id = 'map'
    OXY_ODOM.child_frame_id = 'base_link'
    
     #set first timestamp for delta t
    getTime()
    rospy.sleep(1)

    try:
        while not rospy.is_shutdown():
            TIME_DIF = getTimeDelta()

            if INIT_FILTERED_MAP:
                odometry_ackermann(TIME_DIF,SPEED,STEER)
            else:
                rospy.loginfo('Waiting for filtered_map Odom.')

            rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('KILL')

        