from map import Map
from steering_pid import SteeringPID

import numpy
import rospy

from autominy_msgs.msg import SpeedCommand, SteeringCommand
from nav_msgs.msg import Odometry


		self.lane_1 = np.load("lane1.npy")
		self.lane_2 = np.load("lane2.npy")
		self.lanes = [
		    Lane(self.lane_1[[0, 50, 209, 259, 309, 350, 409, 509, 639, 750, 848, 948, 1028, 1148, 1200, 1276], :]),
		    Lane(self.lane_2[[0, 50, 100, 150, 209, 400, 600, 738, 800, 850, 900, 949, 1150, 1300, 1476], :])]

CALIB_CONST = 100
START_LANE = 0

POS_ODOM = Null
LA_POINT = [Null, Null]
T_ANGLE  = Null
LANES    = Null

def init_map():
    LANES = Map().lanes
    rospy.loginfo("Map loaded!")

def map_callback(msg):


def speed_output(desired_speed):
    SPEED_VAL = SpeedCommand()
    SPEED_VAL.value = desired_speed
    speed_pub.publish(SPEED_VAL)

if __name__ == '__main__': 
    rospy.init_node('denzel_autonomous', anonymous=True)
    rospy.loginfo('Starting...')

    #Publisher for Steering systems
    steer_pub = rospy.Publisher('/control/steering', SteeringCommand, queue_size=10)
    speed_pub = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)

    #Subscribers for Odometry systems
    map_sub = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, map_callback, quere_size=10)
    odom_sub = rospy.Subscriber('/sensors/odometry/odom', Odometry, odom_callback, queue_size=10)
    rospy.sleep(1)

    #set speed accordingly
    speed_output(0.2)

    #set rate of odometry beeing called
    rate = rospy.Rate(RP_RATE)
    
     #set first timestamp for delta t
    getTime()
    rospy.sleep(1)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.sleep(1)
        speed_output(0.0)
        rospy.loginfo('KILL')

        