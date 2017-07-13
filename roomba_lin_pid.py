#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import tf
from math import radians,sqrt

rospy.init_node("roomba_pid_controller")

velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pose_pub = rospy.Publisher('/start_pose', Twist, queue_size=10)

rate = rospy.Rate(10.0)

listener = tf.TransformListener()

def check_camera():
	
	got_one = False

	while not got_one:

		try:
			(t,rot) = listener.lookupTransform('/map', '/roomba', rospy.Time(0))
			got_one = True

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

	euler = tf.transformations.euler_from_quaternion(rot)
	yaw = euler[2]
	quat = tf.transformations.quaternion_from_euler(0.0,0.0,yaw)
	trans = (t[0],t[1],0.0)
	return (trans,quat)

def publish_cmd_vel(lin_vel):
	vel_msg = Twist()
	vel_msg.linear.x = lin_vel
	velocity_publisher.publish(vel_msg)

def publish_start(x,y,d):

	vel_msg = Twist()

	vel_msg.linear.x = x
	vel_msg.linear.y = y
	vel_msg.linear.z = d
	pose_pub.publish(vel_msg)

kp = 1.5
ki = 0
kd = 0

while not rospy.is_shutdown():


	integral = 0
	last_error = 0
	derivative = 0

	i = 0

	target_achieved = False

	target = input("Target Distance [meters]:")

	(trans,rot) = check_camera()
	x_init = trans[0]
	y_init = trans[1]

	publish_start(x_init,y_init,target)

	if target >= 0:
		sign = 1
	if target < 0:
		sign = -1

	while not target_achieved:

		(trans,quat) = check_camera()
		cur_x = trans[0]
		cur_y = trans[1]
		cur_d = sqrt((cur_x - x_init)**2 + (cur_y - y_init)**2)
		error = abs(target) - cur_d
		print "Error: "+ str(error)
		integral += error
		derivative = error - last_error
		lin_vel = kp*error + ki*integral + kd*derivative
		lin_vel *= sign
		print "Lin_vel: " + str(lin_vel)
		if abs(error) < 0.01:
			lin_vel = 0
			i += 1
		publish_cmd_vel(lin_vel)
		last_error = error
		print "---"
		if i >= 5:
			target_achieved = True
		rate.sleep()
