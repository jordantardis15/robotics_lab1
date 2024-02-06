#!/usr/bin/env python3

# imports ROS for developing the node
import rospy

from turtlesim.msg import Pose
# for conversions that may be necessary 
import math

# imports geometry/msgs/Twist for control commands
from geometry_msgs.msg import Twist
# imports controller for turtle
from robotics_lab1.msg import Turtlecontrol

pos_msg = Pose()
cont_msg = Turtlecontrol()

# defining a subscriber callback function
def pose_callback(data):
    global pos_msg
    pos_msg = data

# defining a subscriber callback function
def control_callback(data):
    global cont_msg
    cont_msg = data

if __name__ == '__main__':
	# initializes the node
    rospy.init_node("turtle_controller", anonymous = True)
    # adding subscriber to read the position information
    pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose,pose_callback)
    # adding a subscriber to read Turtlecontrol
    control_subscriber = rospy.Subscriber('/turtle1/control_params',Turtlecontrol,control_callback)
    # declares a publisher to publish in the velocity command topic
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)

    loop_rate = rospy.Rate(10)
    
    # declaring a variable of type Twist for sending control commands
    vel_msg = Twist()

	# run this control loop regularly
    while not rospy.is_shutdown():
    	# Proportional controller equation
        vel_msg.linear.x = cont_msg.kp * (cont_msg.xd - pos_msg.x)
        print(cont_msg.xd)
        print(cont_msg.kp)
        print(cont_msg.xd - pos_msg.x)
        print("")
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
