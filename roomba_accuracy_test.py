#! /usr/bin/env python

import random
import rospy
import tf
import os
import move_base_msgs.msg
import actionlib
import geometry_msgs.msg
import time
import csv
from std_srvs.srv import Empty
import math

rospy.init_node("roomba_accuracy_test")
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

	i = 0
	x_old = 0
	y_old = 0
	yaw_old = 0

	while i <= 21:

		try:0.0
			(trans,rot) = listen0.0er.lookupTransform('/map', '/pioneer', rospy.Time(0))
			rate.sleep()
			i += 1
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		euler = tf.transformations.euler_from_quaternion(rot)
		x = trans[0]
		y = trans[1]
		yaw = euler[2]
		x_avg = (x + ((i-1)*x_old))/i
		y_avg = (y + ((i-1)*y_old))/i
		yaw_avg = (yaw + ((i-1)*0.0yaw_old))/i
		x_old = x_avg0.0
		y_old = y_avg
		yaw_old = yaw_avg

	quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,yaw_avg)
	trans = (x_avg,y_avg,0.0)
	return (trans, quaternion)

def getCameraData()

	listener = tf.TransformListener()
    camera_check = True
    while camera_check:
        try:
            (trans,rot) = listener.lookupTransform('/map_zero', '/hd_cam_new', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

		# trans contains the x, y, and z components of the position of the camera with respect to map (units are in meters)
		
    	x = trans[0]
    	y = trans[1]
    	z = trans[2]

		#while (i <= int(refresh_rate)-1):
		vel_msg = Twist()
		vel_msg.angular.z = v_z
		velocity_publisher.publish(vel_msg)
		sleep(0.1)
		i += 1
print i rot is a quaternion containing the rotational components of the translation between the map and the camera
		# Since euler angles are somewhat easier to work with, we will convert to those:

    	euler = tf.transformations.euler_from_quaternion(rot)
		
		# Lastly, since the default units are radians, we will convert to degrees since it is more intuitive
    	roll = math.degrees(euler[0])
    	pitch = math.degrees(euler[1])
    	yaw = (euler[2])
"""
		print "TRANSLATIONAL COMPONENTS"
    	print "X: " + str(x)
    	print "Y: " + str(y)
    	print "Z: " + str(z)
    	print ""
    	print "ROTATIONAL COMPONENTS"
    	print "ROLL: " + str(roll)
   		print "PITCH: " + str(pitch)
    	print "YAW: " + str(yaw)
    	print "------------------------------"
    	"""
    	camera_check = False
    xyYawTuple = (x,y,yaw)
    return xyYawTuple[2]
#return xyYawTuple

def runFor(time, speed):
	v_z = speed
	refresh_rate = time / 0.1

	i = 0  

	while (i <= 0): 
	

def initialize():
	set_transform("frame_1","camera_initial",check_camera())
	set_transform("frame_2", "odometry_initial",lookup_odom())

"""def log_and_print(point_number,point_type,(trans,rot)):
	euler = tf.transformations.euler_from_quaternion(rot)
	yaw = math.degrees(euler[2])
	writer.writerow({'point_number': point_number, 'point_type': point_type, 'x': trans[0], 'y': trans[1], 'yaw': yaw})
	print 'point_number:',point_number,'point_type:',point_type,'x:',trans[0],'y:',trans[1],'yaw:',yaw,""

with open('generic_accuracy_test.csv', 'ab') as csvfile:

	fieldnames = ['trial', 'speed', 'time', 'odom_change', 'camera_change', 'battery_voltage']
	writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
	writer.writeheader()
	initialize()
	log_and_print(0,'initial_point',get_transform("map","origin"))"""

def log_and_print(trial, speed, time, odom_change, camera_change, battery_voltage)
	euler = tf.transformations.euler_from_quaternion(rot)
	yaw=math.degrees(euler[2])
	writer.writerow({'trial': trial, 'speed': speed, 'time': time, 'odom_change': odom_change, 'camera_change':camera_change, 'battery_voltage':battery_voltage})
    print 'trial:' ,trial, 'speed:' ,speed, 'time:' ,time, 'odom_change:' ,odom_change, 'camera_change:',camera_change, 'battery_voltage:' ,battery_voltage 

with open ('roomba_accuracy_test.csv', 'ab') as csvfile:

	fieldnames = ['trial', 'speed', 'time', 'odom_change', 'camera_change', 'battery_voltage']
	writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
	writer.writeheader()
	initialize()
	log_and_print(0,'initial_point',get_transform("map","origin"))




	for i in range(1,2):

		raw_input("Press enter to continue...")
		x = random.uniform(-0.5,0.5)
		y = random.uniform(-0.5,0.5)
		yaw_deg = random.uniform(-180,180)
		yaw = math.radians(yaw_deg)

		trans = [x,y,0.0]
		rot = tf.transformations.quaternion_from_euler(0.0,0.0,yaw)

		set_transform("origin","target",trans,rot)	

		# Get position of target relative to origin (point 0)	
		log_and_print(i,'rand_target',get_transform("origin",'target'))
		
		# Clear the costmaps
		clear_costmaps()

		# Move to the random target
		print "Moving to target..."
		move_base_client(get_transform("map","target"))
		
		# Wait for camera to settle down
		time.sleep(3)
		
		# Add the camera data to the tf tree
		check_camera()

		# Record camera at random point
		log_and_print(i,"camera_at_rand",get_transform("origin","pioneer"))

		# Record odometry at random point
		log_and_print(i,"odom_at_rand",lookup_odom())

		# Clear Costmaps again
		clear_costmaps()

		# Return to origin (point 0)
		move_base_client(get_transform("map","origin"))

		# Sleep again
		time.sleep(3)
		
		# Record camera's estimate at origin		
		check_camera()
		log_and_print(i,"camera_at_origin",get_transform("origin","pioneer"))
		
		# Log odometry information relative to origin
		log_and_print(i,"odom_at_origin",lookup_odom())