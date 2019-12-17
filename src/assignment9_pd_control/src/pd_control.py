#!/usr/bin/env python

import rospy
from autominy_msgs.msg import SteeringAngle, Speed, NormalizedSteeringCommand, SpeedCommand
from nav_msgs.msg import Odometry
import math
import sys
import tf

RP_RATE = 100

GPS_ID = 999

E_LAST = 0.0

KD = 1.5
KP = 1.0

ANGLE = 0.0
DESIRED_ANGLE = 0.0 #angle can be set during startup

OXY_SUM = [0.0,0.0,0.0,0.0] #ORDER CHANGED

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

def steer_output(desired_angle):
    STEER_VAL = NormalizedSteeringCommand()
    STEER_VAL.value = desired_angle
    steer_pub.publish(STEER_VAL)

def speed_output(desired_speed):
    SPEED_VAL = SpeedCommand()
    SPEED_VAL.value = desired_speed
    speed_pub.publish(SPEED_VAL)

def odom_callback(data):
    global ANGLE

    #setup paths for filtered map odom
    headerGPS = data.pose.pose
    orientationGPS = headerGPS.orientation
    
    #set internal coordinates
    OXY_SUM = [orientationGPS.x, orientationGPS.y, orientationGPS.z,orientationGPS.w]
    _, _, set_yaw = tf.transformations.euler_from_quaternion(OXY_SUM)
    ANGLE = set_yaw

def makeStep(time_passed):
    global E_LAST

    # calculate e(t) = r(t) - y(t)
    if DESIRED_ANGLE > ANGLE:
      E = DESIRED_ANGLE - ANGLE
    else:
      E = ANGLE-DESIRED_ANGLE


    E_DIF = (E-E_LAST)
    STEER = ((KP*E) + (KD*E_DIF))

    # publish steering command
    steer_output(STEER)
    rospy.loginfo(' {}'.format(STEER))

    # for next step
    E_LAST = E

if __name__ == '__main__': 
    rospy.init_node('denzel_pd_control', anonymous=True)
    rospy.loginfo('Starting...')

    #Publisher for Steering systems
    steer_pub = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    speed_pub = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)

    #Subscribers for Odometry systems
    #gps_sub = rospy.Subscriber('/communication/gps/{}'.format(GPS_ID), Odometry, odom_callback, queue_size=10)
    odom_sub = rospy.Subscriber('/sensors/odometry/odom', Odometry, odom_callback, queue_size=10)
    rospy.sleep(1)

    #set desired Angle through input
    try:
        DESIRED_ANGLE = float(sys.argv[1])
    except IndexError:
        rospy.loginfo('NO ANGLE SET')

    speed_output(0.5)
        
    rospy.loginfo('DESIRED ANGLE: {}'.format(DESIRED_ANGLE))

    #set rate of odometry beeing called
    rate = rospy.Rate(RP_RATE)
    
     #set first timestamp for delta t
    getTime()
    rospy.sleep(1)

    try:
        while not rospy.is_shutdown():
            TIME_DIF = getTimeDelta()

            makeStep(TIME_DIF)

            rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.sleep(1)
        speed_output(0.0)
        rospy.loginfo('KILL')

        