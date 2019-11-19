#!/usr/bin/env python

import rospy
#STEERING
from autominy_msgs.msg import SteeringFeedback, SteeringPWMCommand, NormalizedSteeringCommand, SteeringCommand
#SPEEDCONTROL
from autominy_msgs.msg import SpeedCommand, NormalizedSpeedCommand, SpeedPWMCommand, Speed
#COORDINATE SYSTEM
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose

GLOBAL_STEER=0
GLOBAL_SPEED=0.0

##Coordinates
local_coord = [0.0,0.0,0.0]


def steeringCallback(data):
    GLOBAL_STEER=data.value

def gpsCallback(data):
    header = data.pose.pose.position
    local_coord = [header.x,header.y,header.z]
    rospy.loginfo("%s", header)

def speedCallBack(data):
    GLOBAL_SPEED=data.value


rospy.init_node('calibrate_steer_control', anonymous=True)

rate = rospy.Rate(10)
# SUBSCRIBER
subAngle = rospy.Subscriber('/sensors/arduino/steering_angle', SteeringFeedback, steeringCallback, queue_size=10)
subGPS = rospy.Subscriber('/communication/gps/5', Odometry, gpsCallback, queue_size=10)
subSpeed = rospy.Subscriber('/sensors/speed', Speed, speedCallBack, queue_size=10)

#PUBLISHER
pubSteer = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
pubSpeed = rospy.Publisher('/actuators/speed_normalized', NormalizedSpeedCommand)

def turn(val):
    norm_steer=NormalizedSteeringCommand()
    norm_steer.value=val
    return norm_steer

def speed(val):
    norm_speed=NormalizedSpeedCommand()
    norm_speed.value=val
    return norm_speed

def driveCircle(steerVal, speedVal):
    pubSpeed.publish(speed(speedVal))
    pubSpeed.publish(turn(steerVal))


steering_left = turn(1.0) 
steering_middle = turn(0.0)
steering_right = turn(-1.0)

no_speed = 0.0
low_speed = 0.2
mid_speed = 0.5
hig_speed = 1.0

while not rospy.is_shutdown():
    driveCircle(0.05, 0.1)
    rospy.sleep(4)
    pubSpeed.publish(speed(no_speed))
rospy.spin()
rospy.loginfo("DONE")