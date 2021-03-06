#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import os
import time
import tf2_ros

ns = rospy.get_namespace()
print ns
ns = "/roomba/"
t = tf.Transformer(True, rospy.Duration(20.0))
broadcaster = tf2_ros.StaticTransformBroadcaster()

def set_transform(from_a,to_b,x,y,qz,qw):

	m = geometry_msgs.msg.TransformStamped()
	m.header.frame_id = from_a
	m.child_frame_id = to_b
	m.transform.translation.x = x
	m.transform.translation.y = y
	m.transform.rotation.x = 0.0
	m.transform.rotation.y = 0.0
	m.transform.rotation.z = qz
	m.transform.rotation.w = qw
	t.setTransform(m)

def check_for_request():

	got_one = False
	i =0

	while not got_one:
		rate.sleep()
		i+-1
		if i>10:
			break
		try:
			update_msg = rospy.wait_for_message(ns+"update_odom", Empty)
			got_one = True
		except:
			(rospy.exceptions.ROSException,rospy.exceptions.ROSInterruptException)
			continue
		print "trying"
	return got_one

def check_stationary():

	stationary = False
	
	cmd_msg = rospy.wait_for_message("cmd_vel", Twist)
	
	if cmd_msg.linear.x == 0 and cmd_msg.angular.z == 0:
		stationary = True
	
	return stationary

def check_camera():

	i = 0.0
	x_old = 0.0
	y_old = 0.0
	yaw_old = 0.0

	while i <= 21.0:

		try:
			(trans,rot) = listener.lookupTransform('/map', '/roomba', rospy.Time(0))
			rate.sleep()
			i += 1.0
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		euler = tf.transformations.euler_from_quaternion(rot)

		x = trans[0]
		y = trans[1]

		yaw = euler[2]

		x_avg = (x + ((i-1.0)*x_old))/i
		y_avg = (y + ((i-1.0)*y_old))/i
		yaw_avg = (yaw + ((i-1.0)*yaw_old))/i

		x_old = x_avg
		y_old = y_avg
		yaw_old = yaw_avg
		
	quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,yaw_avg)

	set_transform("map_guess","camera_guess",x_avg,y_avg,quaternion[2],quaternion[3])
	
def Main():

	running = False
	got_one = True
	while not rospy.is_shutdown():
		
		if running:
			print "hello"
			got_one = check_for_request()
			print "Update requested"
		if got_one:
			try:
				(trans,rot) = listener.lookupTransform('base_link', 'odom', rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			set_transform("camera_guess","temp_odom",trans[0],trans[1],rot[2],rot[3])

			print "Not moving, estimating position"
			check_camera()
			if running:
				print "Killing previous transform"
			(trans,quat) = t.lookupTransform("map_guess","temp_odom",rospy.Time(0))
			print "Publishing tf map -> odom"
			
			static_transformStamped = geometry_msgs.msg.TransformStamped()
		
			static_transformStamped.header.stamp = rospy.Time.now()
			static_transformStamped.header.frame_id = "/map"
			static_transformStamped.child_frame_id = "odom"
		
			static_transformStamped.transform.translation.x = trans[0]
			static_transformStamped.transform.translation.y = trans[1]
			static_transformStamped.transform.translation.z = 0.0

			static_transformStamped.transform.rotation.x = 0.0
			static_transformStamped.transform.rotation.y = 0.0
			static_transformStamped.transform.rotation.z = quat[2]
			static_transformStamped.transform.rotation.w = quat[3]
		
			broadcaster.sendTransform(static_transformStamped)
			running = True
			time.sleep(3)
		"""
		else:
			print "The robot is still moving, please try again later"
		"""
if __name__ == '__main__':
	rospy.init_node('odom_reset')
	listener = tf.TransformListener()
	rate = rospy.Rate(15.0)
	Main()
