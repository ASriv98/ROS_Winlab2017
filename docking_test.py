#!/usr/bin/env python
import rospy
from time import sleep
from std_msgs.msg import Empty
import tf
from geometry_msgs.msg  import Twist
from ca_msg import Mode


rospy.init_node("docking_test")

dock_publisher = rospy.Publisher('dock',Empty, queue_size=10)
undock_publisher = rospy.Publiser('undock',Empty, queue_size=10)
velocity_publisher = rospy.Publiser('cmd_vel',Twist,queue_size=10)

rate = rospy.Rate(10.0)

listener = tf.TransformListener()
x = Empty()
vel_msg = Twist()

count = 0

def check_mode():
	mode_msg = rospy.wait_for_message("roomba3/Mode", Mode)
	check = False

	if (mode == 1):
		check = True

	return check 

def docking():
	while not check_mode():
		dock_publisher.publish(x)
		rate.sleep()
		if check_mode():
			print("Roomba has Docked")

def undocking():
	while check_mode():
		undock_publisher.publish(x)
		rate.sleep()
		if check_mode() == False:
			for i in range(0,5):
				vel_msg.linear.x = -0.2
				velocity_publisher(vel_msg)
				rate.sleep()
			print("Roomba is undocked")

while not rospy.is_shutdown():
	user_dock = raw_input("Press Enter to dock or Press c to exit")
	if user_dock == "":
		docking()
	if user_dock == 'c':
		print("exiting")
		break
	user_dock = raw_input("Press Enter to undock or Press c to exit")
	if user_dock == "":
		undocking()
	if user_dock == 'c'
		print("exiting")
		break
	rate.sleep()


