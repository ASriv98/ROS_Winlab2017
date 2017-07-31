#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from std_msgs.msg import Bool
from math import pow,atan2,sqrt
from nav_msgs.msg import Odometry
import tf
from math import radians, degrees, sqrt
from time import sleep
from numpy import sign
from ca_msgs.msg import Bumper
from breezycreate2 import Robot
import time
import csv


rospy.init_node("roomba_pid_controller")

velocity_publisher = rospy.Publisher('roomba3/cmd_vel', Twist, queue_size=10)
disable_publisher = rospy.Publisher('disable_driver',Bool,queue_size=10)

rate = rospy.Rate(10.0)

listener = tf.TransformListener()


def check_camera():
	
	got_one = False

	while not got_one:

		try:
			(t,rot) = listener.lookupTransform('/map', '/roomba3', rospy.Time(0))
			got_one = True

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

	euler = tf.transformations.euler_from_quaternion(rot)
	yaw = euler[2]
	quat = tf.transformations.quaternion_from_euler(0.0,0.0,yaw)
	trans = (t[0],t[1],0.0)
	return (trans,quat)

def publish_cmd_vel(lin_vel, angle_vel):
	vel_msg = Twist()
	vel_msg.linear.x = lin_vel
	vel_msg.angular.z = angle_vel
	velocity_publisher.publish(vel_msg)

def bumperData():
	bumper_msg = rospy.wait_for_message("roomba3/bumper", Bumper)
	check = False

	if (bumper_msg.is_left_pressed or bumper_msg.is_right_pressed):
		check = True

	return check 

def pulse(speed):
	bot = Robot()
	bot.playNote('A4', 5)
	bot.setTurnSpeed(speed)
	time.sleep(0.1)
	bot.setTurnSpeed(0)
	bot.close()

def step(speed):
	bot = Robot()
	bot.playNote('A4',5)
	bot.setForwardSpeed(speed)
	time.sleep(0.1)
	bot.setForwardSpeed(0)
	bot.close()

def moveTo(distance):
	accel = 0.01
	kp = 0.5
	ki = 0
	kd = 0
	integral = 0
	last_error = 0
	derivative = 0

	i = 0

	target_achieved = False

	target = distance 
	(trans,rot) = check_camera()
	x_init = trans[0]
	y_init = trans[1]


	if target >= 0:
		sign = 1
	if target < 0:
		sign = -1
	cur_speed = 0

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
		publish_cmd_vel(lin_vel,0)
		last_error = error
		print "---"
		if i >= 5:
			target_achieved = True
		rate.sleep()

	sleep(3)
	(trans,quat) = check_camera()
	cur_x = trans[0]
	cur_y = trans[1]
	cur_d = sqrt((cur_x - x_init)**2 + (cur_y - y_init)**2)
	error = abs(target) - cur_d

	cmd = Bool()
	cmd.data = True
	for i in range(5):
		disable_publisher.publish(cmd)
		rate.sleep()

	failed = 0
	speed = 25
	stop_d = 0.01
	last_error = error

	while abs(error)>=stop_d:

		(trans,quat) = check_camera()
		euler = tf.transformations.euler_from_quaternion(quat)
		cur_x = trans[0]
		cur_y = trans[1]
		cur_d = sqrt((cur_x - x_init)**2 + (cur_y - y_init)**2)
		error = abs(target) - cur_d

		if error > 0:
			sign = -1
		else:
			sign = 1
		derivative = error - last_error
		if abs(derivative) <= 0.003:
			failed += 1
			stop_d = 0.01
		else:
			failed = 0
		if failed >= 5:
			speed += 1
			failed = 0
		step(speed*sign)
		if abs(derivative) >= stop_d:
			stop_d = abs(derivative)
		#ang_vel = (error/radians(180))*0.35
		print "Error: "+ str(error)
		print "Derivative: " + str(derivative)
		print "Speed: " + str(speed)
		print "Failed Tries: " + str(failed)
		print "--"
		last_error = error
		rate.sleep()
	cmd.data = False
	for i in range(5):
		disable_publisher.publish(cmd)
		rate.sleep()

	sleep(3)
	(trans,quat) = check_camera()
	cur_x = trans[0]
	cur_y = trans[1]
	cur_d = sqrt((cur_x - x_init)**2 + (cur_y - y_init)**2)
	error = abs(target) - cur_d
	print "Final Error: " + str(fe)

def rotateTo(angle):
	#kp_max = 1.6
	#kp_min = 0.2
	kp = 0.8
	ki = 0
	kd = 0

	window_max = 0.08
	window_min = 0.05

	window = 0.35

	accel = 0.001

	integral = 0
	last_error = 0
	derivative = 0

	sleep(2)

	i = 0

	cur_speed = 0.25

	target_achieved = False
	(trans,quat) = check_camera()
	euler = tf.transformations.euler_from_quaternion(quat)
	yaw = euler[2]
	print degrees(yaw)
	
	target = angle

	error = target - yaw

	cur_speed *= sign(error)

	
	#window = window_max - ((abs(error)/radians(180))*(window_max-window_min))

	while not target_achieved:

		(trans,quat) = check_camera()
		euler = tf.transformations.euler_from_quaternion(quat)
		yaw = euler[2]
		error = target - yaw
		if error > radians(180):
			error = error - radians(360)
		if error < radians(-180):
			error = error + radians(360)
		#ang_vel = (error/radians(180))*0.35
		print "Error: "+ str(error)
		#kp = kp_min + ((abs(error)/radians(180))*(kp_max-kp_min))
		#ang_vel *= sign(error)
		#ang_vel = round(ang_vel,3)
		#print "Kp: "+str(kp)

		integral += error
		derivative = error - last_error
		ang_vel = kp*error + ki*integral + kd*derivative
		#print "Set Point for Speed: " + str(ang_vel)
	
		if ang_vel > 0 and ((ang_vel-cur_speed)>accel):
			ang_vel = cur_speed + accel
		if ang_vel < 0 and ((cur_speed-accel) > ang_vel):
			ang_vel = cur_speed - accel
	
		cur_speed = ang_vel
	
		print "Target angle: " + str(degrees(target))
		print "Window: "+ str(window)
		if abs(error) <= window: #old threshold =0.006
			for j in range(0,10):
				ang_vel = 0
				publish_cmd_vel(0,ang_vel)
				rate.sleep()
			target_achieved = True
		print "Ang_vel: " + str(ang_vel)
		publish_cmd_vel(0,ang_vel)
		if (last_error >0 and error < 0) or (last_error<0 and error>0) or (abs(error)<0.01):
			integral = 0
		last_error = error
		print "---"

		rate.sleep()

	cmd = Bool()
	cmd.data = True
	for i in range(5):
		disable_publisher.publish(cmd)
		rate.sleep()
	failed = 0
	speed = 30
	stop_d = radians(0.4)
	(trans,quat) = check_camera()
        euler = tf.transformations.euler_from_quaternion(quat)
	yaw = euler[2]
	error = target - yaw
	last_error = error
	while abs(error)>=stop_d:
		turn = 1
		(trans,quat) = check_camera()
		euler = tf.transformations.euler_from_quaternion(quat)
		yaw = euler[2]
		error = target - yaw
		if error > 0:
			turn = -1
		else:
			turn = 1
		if error > radians(180):
			error = error - radians(360)
		if error < radians(-180):
			error = error + radians(360)
		derivative = error - last_error
		if abs(derivative) <= 0.005:
			failed += 1
			stop_d = radians(0.04)
		else:
			failed = 0
		if failed >= 5:
			speed += 1
			failed = 0
		pulse(speed*turn)
		if abs(derivative) >= stop_d:
			stop_d = abs(derivative)
		#ang_vel = (error/radians(180))*0.35
		print "Error: "+ str(error)
		print "Derivative: " + str(derivative)
		print "Speed: " + str(speed)
		print "Failed Tries: " + str(failed)
		print "--"
		last_error = error
		rate.sleep()
	cmd.data = False
	for i in range(5):
		disable_publisher.publish(cmd)
		rate.sleep()

	sleep(3)
	(trans,quat) = check_camera()
	euler = tf.transformations.euler_from_quaternion(quat)
	yaw = euler[2]
	fe = target - yaw
	print "Final Error: " + str(fe)
	print degrees(fe)



def moveBreak(target_x,target_y):

	(trans, rot) = check_camera()
	x_init = trans[0]
	y_init = trans[1]
	
	points = []

	total_x = target_x-x_init
	total_y = target_y-y_init
	distance = sqrt((total_x)**2+(total_y)**2)
	step = 1
	d = step

	while d < distance:

		step_x = (d*total_x)/distance
		step_y = (d*total_y)/distance
		x = x_init + step_x
		y = y_init + step_y
		points.append([x,y])
		d += step

	points.append([target_x,target_y])
	print points
	return points


def input_points():
        with open('points.csv') as csvfile:
                points_reader = csv.reader(csvfile, delimiter = ',')
                for row in points_reader:
                        moveBreak(row[0], row[1])


def move2goal():

	(trans, rot) = check_camera()
	current_x = trans[0]
	current_y = trans[1]

	target_x = input("Set your x: ")  
	target_y = input("Set your y: ")
	points = moveBreak(target_x,target_y)

	for point in points: 

		point_x = point[0]
		point_y = point[1]

		(trans, rot) = check_camera()
		current_x = trans[0]
		current_y = trans[1]
		direction = atan2((point_y-current_y),(point_x-current_x))
		print degrees(direction)


		rotateTo(direction)
		sleep(2)
		
		(trans, rot) = check_camera()
		current_x = trans[0]
		current_y = trans[1]
		movement_distance = sqrt((point_x-current_x)**2+(point_y-current_y)**2)
		moveTo(movement_distance)


while not rospy.is_shutdown():
	print check_camera()
	move2goal()
