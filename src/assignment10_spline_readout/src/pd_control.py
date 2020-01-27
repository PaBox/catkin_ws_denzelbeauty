#!/usr/bin/env python

import rospy
import numpy
import random
import math
import sys
import tf

from autominy_msgs.msg import SteeringAngle, Speed, NormalizedSteeringCommand, SpeedCommand
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

DRIVE_SPLINE_1 = numpy.load('lane1.npy')
DRIVE_SPLINE_2 = numpy.load('lane2.npy')


def generate_support_points(quadrant):
    quadrant_length = len(quadrant)
    random_points = []

    for range(10):  
        random_points.append(
            quadrant[
                random.randint(
                    1,quadrant_length
                )-1
            ]
        )

    return random_points

def analyze_drive_spline(spline):
    # top, left, bottom, right
    Q1, Q2, Q3, Q4 = []

    for point in spline:
        int_arc = int(point[0])
        if   9 <= int_arc < 13:
            Q4.append(point)
        elif 0 <= int_arc < 3:
            Q1.append(point)
        elif 3 <= int_arc < 6:
            Q2.append(point)
        elif 6 <= int_arc < 9:
            Q3.append(point)

    reduced_q1 = generate_support_points(Q1)
    reduced_q2 = generate_support_points(Q2)
    reduced_q3 = generate_support_points(Q3)
    reduced_q4 = generate_support_points(Q4)
    
    points = np.concatenate([reduced_q1,reduced_q2,reduced_q3,reduced_q4])
    points = points[points[:,0].argsort()]
    # 5 random gewaehlte Punkte aus jedem Quadranten
    return points

if __name__ == '__main__': 
    rospy.init_node('denzel_spline_readout', anonymous=True)
    rospy.loginfo('Starting...')

    #Publisher for Steering systems
    spline_marker_pub = rospy.Publisher('/spline/marker', Marker, queue_size=10)
    closest_point_pub = rospy.Publisher('/closest/point', Marker, queue_size=10)
    lookahead_point_pub = rospy.Publisher('/lookahead/point', Marker, queue_size=10)

    rospy.sleep(1)

    analyze_drive_spline(DRIVE_SPLINE_1)

    #set rate of odometry beeing called
    rate = rospy.Rate(RP_RATE)

    try:
        while not rospy.is_shutdown():
            

            rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.sleep(1)
        rospy.loginfo('KILL')

        