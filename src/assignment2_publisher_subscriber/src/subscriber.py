#!/usr/bin/env python
import rospy
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand

def callbackSpeed(data):
	rospy.loginfo(
		"\nThe Actual speed is %s", data.value
	)

rospy.init_node('speedSubscriber', anonymous=True)

rospy.Subscriber("/sensors/speed", SpeedCommand, callbackSpeed)

rospy.spin()

