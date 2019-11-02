#!/usr/bin/env python
import rospy
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand

rospy.init_node('speedPublisher', anonymous=True)

pubSt = rospy.Publisher('actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)

pubSp = rospy.Publisher('actuators/speed', SpeedCommand, queue_size=10)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
	rospy.loginfo("Publishing Speed...please be patient.")
	pubSp.publish(value=0.4)
	pubSt.publish(value=1.0)
	rate.sleep()


