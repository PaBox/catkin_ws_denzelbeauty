#!/usr/bin/env python

import rospy
import math
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

def turnRadius(radius):
    return math.degrees(math.atan(0.27/radius))

def setSteeringAngle(ang):
    pubSteer.publish((float(abs(335-ang.value))/250))
    return

def calcRadiusFrom((x1,y1),(x2,y2),(x3,y3)):
    if x2-x1==0.0:
        x=x+0.01
    if x3-x1==0.0:
        x=x+0.01
    if y2-y1==0.0:
        x=x+0.01
    if y3-y1==0.0:
        y=y+0.01

    angle1 = -(1/((y2-y1)/(x2-x1)))
    angle2 = -(1/((y3-y1)/(x3-x1)))

    if angle1-angle2 == 0.0:
        angle2 = angle1 + 0.01

    relX = (-(angle1*x2)+y2+(angle2*x3)-y3)/(angle2-angle1)
    relY = (angle1*relX)+(angle1*(-x2))+y2

    return (math.sqrt((x1-relX)**2))/2

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

#get first point and drive to second position
first_position = (local_coord[0],local_coord[1])
pubSteer.publish(steering_left)
rospy.sleep(1)
pubSpeed.publish(low_speed)
rospy.sleep(4)
pubSpeed.publish(no_speed)
rospy.sleep(1)

#get second point and drive to the third position
second_position = (local_coord[0],local_coord[1])
pubSpeed.publish(low_speed)
rospy.sleep(4)
pubSpeed.publish(no_speed)
rospy.sleep(1)

#get third position
third_position = (local_coord[0],local_coord[1])

#print out formatted outputturnRadius
radius = turnRadius (
    first_position,
    second_position,
    third_position
)

print ("The driven radius is around %i meter.", radius)

print ("The cars angle is at %i degree.", calcRadiusFrom(radius))

rospy.spin()
rospy.loginfo("DONE")