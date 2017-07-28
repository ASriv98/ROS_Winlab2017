#!/usr/bin/env python
import rospy
from time import sleep
import tf
from geometry_msgs.msg  import Twist
from ca_msg import Mode


rospy.init_node("docking_test")

dock_publisher = rospy.Publisher('dock', queue_size=10)
undock_publisher = rospy.Publiser('undock',queue_size=10)
velocity_publisher = rospy.Publiser('cmd_vel',Twist,queue_size=10)

rate = rospy.Rate(10.0)

listener = tf.TransformListener()

vel_msg = Twist()

count = 0

def check_mode():
	mode_msg = rospy.wait_for_message("roomba3/Mode", Mode)
	check = False

	if (mode == 1):
		check = True

	return check 

while not rospy.is_shutdown():
	while not check_mode():
		dock_publisher.publish()
		rate.sleep()
	if check_mode() == False:
		undock_publisher.publish()
		rate.sleep(2)
		vel_msg.linear.x = -0.2
		velocity_publisher.publish(vel_msg)
		rate.sleep(2)
	count+=1
	if count >= 50:
		break

