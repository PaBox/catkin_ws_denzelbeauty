#!/usr/bin/env python

#GENERAL LIBRARIES
import rospy
import math
#CONTROL
from autominy_msgs.msg import SteeringFeedback, NormalizedSteeringCommand, NormalizedSpeedCommand, Speed, Tick
#COORDINATE SYSTEM
from nav_msgs.msg import Odometry

#DEFINE CLASS VARIABLES - LOGGING
X = 0.0
Y = 0.0
SPEED = 0.0
STEER = 0.0
TICKS = 0
LAST_POS = [0.0,0.0]
DISTANCE = 0.0
BASESPEED = []
MEASURING = False
ODOM_ON = False
GPS_ON = False
GPS_NUMBER = 5

    #CALLBACK FUNCTIONS
def steeringCallback(data):
    global STEER
    STEER=data.value

def gpsCallback(data):
    global GPS_ON
    global X,Y

    if not GPS_ON:
        GPS_ON = True
    
    header = data.pose.pose.position
    X=header.x;Y=header.y

def speedCallback(data):
    global SPEED
    SPEED=data.value

def tickCallback(data):
    global LAST_POS
    global TICKS
    global DISTANCE
    global X,Y
    global BASESPEED
    global SPEED

    TICKS=data.header.seq
    if MEASURING:
        DISTANCE += (math.sqrt(
            (X - LAST_POS[0])**2 +
            (Y - LAST_POS[1])**2))
        BASESPEED.extend(
            [SPEED]) 
    LAST_POS = [X,Y]
    
def odomCallback(data):
    global LAST_POS
    global ODOM_ON
    global X,Y

    if not ODOM_ON:
        ODOM_ON = True
    
    header = data.pose.pose.position
    X=header.x;Y=header.y

#DRIVING FUNCTIONS
def steer(val):
    norm_steer=NormalizedSteeringCommand()
    norm_steer.value=val
    return norm_steer

def speed(val):
    norm_speed=NormalizedSpeedCommand()
    norm_speed.value=val
    return norm_speed

def drive(distance, driveSpeed, angle, testRate):
    global TICKS
    global MEASURING
    global DISTANCE
    global SPEED
    global BASESPEED
    global X,Y
    global LAST_POS

    # stop the car and set desired steering angle + speed
    
    pubSpeed.publish(speed(0.0))
    rospy.sleep(1)
    pubSteer.publish(steer(angle))
    rospy.sleep(1)

    #set start parameters
    MEASURING = True
    first_pos = [X,Y]

    while not rospy.is_shutdown() and DISTANCE < distance:
        pubSpeed.publish(speed(driveSpeed))

    pubSpeed.publish(speed(0.0))
    rospy.sleep(1)
    last_pos = [X,Y]
    MEASURING = False

    rospy.loginfo(
        '\n{}mastered {} meters\nfrom {}\nto {},\nat a base Speed of {}.'.format(
            rospy.get_caller_id(),
            DISTANCE,
            first_pos,
            last_pos,
            (sum(BASESPEED)/len(BASESPEED))
        )
    )

    return DISTANCE

#switch from GPS to ODOM
def initCoordinateSystem():
    rospy.loginfo("Initialising distance tracking system...")
    rospy.sleep(2)
    if not GPS_ON:
        rospy.loginfo("No GPS found, switching to ODOM")
        while not rospy.is_shutdown() and not ODOM_ON:
            rospy.loginfo(
                "%s: No initial odometry message received. Waiting for message...",
            rospy.get_caller_id())
            rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        #INIT NODE
        rospy.init_node("DistanceCalculation", anonymous=False)

        #INIT CLASS VARIABLES
        rate = rospy.Rate(10)

        #ADD SUBSCRIBER
        subAngle = rospy.Subscriber('/sensors/arduino/steering_angle', SteeringFeedback, steeringCallback, queue_size=10)
        subGPS = rospy.Subscriber('/communication/gps/{}'.format(GPS_NUMBER), Odometry, gpsCallback, queue_size=10)
        subSpeed = rospy.Subscriber('/sensors/speed', Speed, speedCallback, queue_size=10)
        subTicks = rospy.Subscriber('/sensors/arduino/ticks', Tick, tickCallback, queue_size=10)
        sub_odom = rospy.Subscriber("/sensors/odometry/odom", Odometry, odomCallback, queue_size=10)

        #ADD PUBLISHER
        pubSteer = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
        pubSpeed = rospy.Publisher('/actuators/speed_normalized', NormalizedSpeedCommand, queue_size=10)
        rospy.sleep(1)

        #Init right Coordination System
        initCoordinateSystem()

        #CALL DRIVING FUNCTION
        #drive(1.0, 0.2, 1.0, rate) #Left
        drive(1.0, 0.2, 0.0, rate) #Straight
        #drive(1.0, 0.2, -1.0, rate) #Right
		
    except rospy.ROSInterruptException:
        pass