#!/usr/bin/env python

import rospy
from autominy_msgs.msg import SteeringFeedback, SteeringPWMCommand

def steeringCallback(data):
    rospy.loginfo("%s", data.value)

rospy.init_node('pwm_sensor_readout', anonymous=True)
subAngle = rospy.Subscriber('/sensors/arduino/steering_angle', SteeringFeedback, steeringCallback, queue_size=10)
pubPWM = rospy.Publisher('/actuators/steering_pwm', SteeringPWMCommand, queue_size=10)
n=0
while n<1000:
    n=n+1
    pubPWM.publish(value=1000+n)
    rospy.loginfo("N IS %i", (1000+n))
    
rospy.sleep(1)
n=0
while n<1000:
    n=n+1
    pubPWM.publish(value=2000-n)
    rospy.loginfo("N IS %i", (2000-n))
rospy.sleep(1)
n=0
rospy.loginfo("DONE")