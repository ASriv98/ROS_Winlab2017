#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import tf
from math import radians, degrees
from numpy import sign
from time import sleep
from breezycreate2 import Robot
import time

rospy.init_node("roomba_pid_controller")

velocity_publisher = rospy.Publisher('/roomba3/cmd_vel', Twist, queue_size=0)
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

def publish_cmd_vel(ang_vel):
	vel_msg = Twist()
	vel_msg.angular.z = ang_vel
	velocity_publisher.publish(vel_msg)

def publish_PID_vel(left_wheel, right_wheel):
	vel_msg = Twist()
	vel_msg.linear.x = left_wheel
	vel_msg.linear.y = right_wheel
	velocity_publisher.publish(vel_msg)
def pulse(sign):
	bot = Robot()
	bot.playNote('A4', 5)
	bot.setTurnSpeed(30*sign)
	time.sleep(0.1)
	bot.setTurnSpeed(0)
	bot.close()

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

while not rospy.is_shutdown():

	sleep(2)

	i = 0

	cur_speed = 0.25

	target_achieved = False
	(trans,quat) = check_camera()
	euler = tf.transformations.euler_from_quaternion(quat)
	yaw = euler[2]
	print degrees(yaw)
	t = input("Target Angle [degrees]:")

	target = radians(t)

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
				publish_cmd_vel(ang_vel)
				rate.sleep()
			target_achieved = True
		print "Ang_vel: " + str(ang_vel)
		publish_cmd_vel(ang_vel)
		if (last_error >0 and error < 0) or (last_error<0 and error>0) or (abs(error)<0.01):
			integral = 0
		last_error = error
		print "---"

		rate.sleep()
	
	cmd = Bool()
	cmd.data = True
	disable_publisher.publish(cmd)
	while abs(error)>=radians(0.4):
		sign =1
		if error>0:
			sign = -1
		else: 
			sign = 1
		pulse(sign)
		rate.sleep()
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
	cmd.data = False
	disable_publisher.publish(cmd)


	


	sleep(3)
	(trans,quat) = check_camera()
	euler = tf.transformations.euler_from_quaternion(quat)
	yaw = euler[2]
	fe = target - yaw
	print "Final Error: " + str(fe)
	print degrees(fe)
