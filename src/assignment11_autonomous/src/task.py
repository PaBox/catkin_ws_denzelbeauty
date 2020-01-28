from map import Map
from steering_pid import SteeringPID

import numpy
import rospy

from visualization_msgs.msg import Marker
from autominy_msgs.msg import SpeedCommand, SteeringCommand
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool


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

def create_lanes_sub():
    la1 = rospy.Subscriber("/point/lookahead1", Marker, la1_callback, quere_size=10)
    la2 = rospy.Subscriber("/point/lookahead2", Marker, la2_callback, quere_size=10)
    return [la1, la2]

def map_callback(odom):
    global POS_ODOM
    
    POS_ODOM = odom

def la1_callback():
    lookahead_callback(msg, 0)

def la2_callback():
    lookahead_callback(msg, 1)

def lookahead_callback(point, lane):
    global LA_POINT

    position = marker.pose.position

    LA_POINT[lane] = (position.x, position.y)

    if lane == START_LANE:
        get_TA_ANGLE()

def lane_callback(nr):
    global START_LANE

    ## Callback for lane change
    ## prevents invalid numbers from interfering with other functions
    if (nr == 0 || nr == 1) {
        START_LANE=0
    }

def quaternion():
    orientation = POS_ODOM.pose.pose.orientation
    
    return [
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    ]

def speed_callback(msg):
    pass ## EMPTY

def speed_output(desired_speed):
    SPEED_VAL = SpeedCommand()
    SPEED_VAL.value = desired_speed
    speed_pub.publish(SPEED_VAL)

def get_TA_ANGLE(self):
global POS_ODOM, LA_POINT, START_LANE

    ## only activate if position is given
    if POS_ODOM is None: return

    msg = Float64()
    position = POS_ODOM.pose.pose.position
    pos_now = np.array([position.x, position.y])

    ## get points
    x = LA_POINT[START_LANE][0] - POS_ODOM[0]
    y = self.lookahead_point[START_LANE][1] - POS_ODOM[1]

    s = atan2(y,x)

    _,_,t = euler_from_quaternion(quaternion())
    print("Calculated angle: [sigma: {s}, theta:{t}]")

    difference = (s - t)
    
    msg.data = difference
    angle_pub.publish(msg)
    steer_pub.publish(steer)

    rospy.loginfo("New target angle is: ", msg.data)

if __name__ == '__main__': 
    rospy.init_node('denzel_autonomous', anonymous=True)
    rospy.loginfo('Starting...')

    lanes_sub = create_lanes_sub()

    #Publisher for Steering systems
    steer_pub = rospy.Publisher('/actuators/steering', SteeringCommand, queue_size=10)
    speed_pub = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)

    #Subscribers for Odometry systems
    speed_sub = rospy.Subscriber('/sensors/speed', Speed, speed_callback, queue_size=10)
    map_sub = rospy.Subscriber('/sensors/localization/filtered_map', Odometry, map_callback, quere_size=10)
    lane_sub = rospy.Subscriber('/position/lane', Int16, lane_callback, queue_size=10)
    rospy.sleep(1)

    #set speed accordingly
    speed_output(0.2)

    #set rate of odometry beeing called
    rate = rospy.Rate(RP_RATE)
    
    rospy.sleep(1)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.sleep(1)
        speed_output(0.0)
        rospy.loginfo('KILL')

        