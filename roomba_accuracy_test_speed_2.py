#!/usr/bin/env python

import roslib; roslib.load_manifest('irobot_create_2_1')
from irobot_create_2_1.msg import SensorPacket
import rospy
import tf
import os
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import time
import csv
import math
from time import sleep

velocities = [0.8,1,2,3,4.5]
times = [0.1,0.25,0.5,0.75,1]

fixed_velocity = 2

rospy.init_node("roomba_accuracy_test")
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

rate = rospy.Rate(10.0)


listener = tf.TransformListener()
t = tf.Transformer(True, rospy.Duration(20.0))

def get_transform(from_a,to_b):

	(trans,rot) = t.lookupTransform(from_a,to_b,rospy.Time(0))
	return (trans,rot)

def set_transform(from_a,to_b,(trans,rot)):

	m = geometry_msgs.msg.TransformStamped()
	m.header.frame_id = from_a
	m.child_frame_id = to_b
	m.transform.translation.x = trans[0]
	m.transform.translation.y = trans[1]
	m.transform.rotation.x = 0.0
	m.transform.rotation.y = 0.0
	m.transform.rotation.z = rot[2]
	m.transform.rotation.w = rot[3]
	t.setTransform(m)

def lookup_odom():

	got_one = False

	while not got_one:
		
		try:
			(trans,rot) = listener.lookupTransform("/odom", "/base_link", rospy.Time(0))
			got_one = True
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		rate.sleep()
		return (trans, rot)

def check_camera():

	i = 0.0000
	x_old = 0.0
	y_old = 0.0
	yaw_old = 0.0

	while i <= 21.0000:

		try:
			(trans,rot) = listener.lookupTransform('/map', '/roomba', rospy.Time(0))
			rate.sleep()
			i += 1.000
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		euler = tf.transformations.euler_from_quaternion(rot)
		x = trans[0]
		y = trans[1]
		yaw = euler[2]

		x_avg = float((x + ((i-1)*x_old))/i)
		y_avg = float((y + ((i-1)*y_old))/i)
		yaw_avg = float(yaw + ((i-1)*yaw_old))/i
		x_old = x_avg
		y_old = y_avg
		yaw_old = yaw_avg

	quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,yaw_avg)
	trans = (x_avg,y_avg,0.0)

	return(trans,quaternion)

def runFor(time, speed):
	v_z = speed
	vel_msg = Twist()
	vel_msg.angular.z = v_z
	velocity_publisher.publish(vel_msg)
	sleep(time)
	vel_msg.angular.z = 0

	for i in range(0,5):
		velocity_publisher.publish(vel_msg)
		rate.sleep()


def initialize():
	set_transform("camFrame","camera_initial",check_camera())
	set_transform("odomFrame", "odometry_initial",lookup_odom())

def lookup_battery():
	sensor_msg = rospy.wait_for_message("sensorPacket", SensorPacket)
	return sensor_msg.voltage
		
	
def log_and_print(trial, speed, time, odom_change, camera_change, current_battery):
	odom_euler = tf.transformations.euler_from_quaternion(odom_change[1])
	odom_yaw = odom_euler[2]
	camera_euler = tf.transformations.euler_from_quaternion(camera_change[1])
	camera_yaw = camera_euler[2]
	writer.writerow({'trial': trial, 'speed': speed, 'time': time, 'odom_change': odom_yaw, 'camera_change': camera_yaw, 'current_battery': current_battery})
	print 'trial:' ,trial, 'speed:' ,speed, 'time:' ,time, 'odom_change:' ,odom_yaw, 'camera_change:', camera_yaw, 'current_battery', current_battery

with open ('Roomba_Accuracy_Test_Trial3.csv', 'ab') as csvfile:

	fieldnames = ['trial', 'speed', 'time', 'odom_change', 'camera_change', 'current_battery']
	writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
	writer.writeheader()
	initialize()
	sleep(2)

	for time in times:

                for trial in range(0, 10):
                        
			runFor(time, fixed_velocity)
			sleep(1.5)
			set_transform("odomFrame", "odometry_final", lookup_odom())
			set_transform("camFrame", "camera_final", check_camera())
			log_and_print(trial, fixed_velocity, time, get_transform("odometry_initial", "odometry_final"), get_transform("camera_initial", "camera_final"), lookup_battery())
			set_transform("camFrame", "camera_initial", get_transform("camFrame", "camera_final"))
			set_transform("odomFrame", "odometry_initial", get_transform("odomFrame", "odometry_final"))
			sleep(1)

			#print trial
