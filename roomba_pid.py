#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import tf
from math import radians

rospy.init_node("roomba_pid_controller")

velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

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

def publish_cmd_vel(ang_vel):
	vel_msg = Twist()
	vel_msg.angular.z = ang_vel
	velocity_publisher.publish(vel_msg)

kp = 0.955
ki = 0
kd = 0

integral = 0
last_error = 0
derivative = 0

while not rospy.is_shutdown():

	i = 0

	target_achieved = False

	t = input("Target Angle [degrees]:")

	target = radians(t)

	while not target_achieved:

		(trans,quat) = check_camera()
		euler = tf.transformations.euler_from_quaternion(quat)
		yaw = euler[2]
		error = target - yaw
		print "Error: "+ str(error)
		integral += error
		derivative = error - last_error
		ang_vel = kp*error + ki*integral + kd*derivative
		print "Ang_vel" + str(ang_vel)
		if abs(error) < 0.006:
			ang_vel = 0
			i += 1
		publish_cmd_vel(ang_vel)
		last_error = error
		print "---"
		if i >= 5:
			target_achieved = True
		rate.sleep()
