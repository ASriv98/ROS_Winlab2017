#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
#from turtlesim.msg import Pose
from math import pow,atan2,sqrt
from nav_msgs.msg import Odometry
import tf
from math import radians, degrees, sqrt
from time import sleep
from ca_msgs.msg import Bumper


rospy.init_node("roomba_pid_controller")

velocity_publisher = rospy.Publisher('roomba3/PID_vel', Twist, queue_size=10)

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

def publish_PID_vel(left_wheel, right_wheel):
	vel_msg = Twist()
	vel_msg.linear.x = left_wheel
	vel_msg.linear.y = right_wheel
	velocity_publisher.publish(vel_msg)

def bumperData():
	bumper_msg = rospy.wait_for_message("roomba3/bumper", Bumper)
	check = False

	if (bumper_msg.is_left_pressed or bumper_msg.is_right_pressed):
		check = True

	return check 

def moveTo(distance):
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

	while not target_achieved:
		check = bumperData()

		if check == True:
			break

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
		publish_PID_vel(lin_vel,lin_vel)
		last_error = error
		print "---"
		if i >= 5:
			target_achieved = True
		rate.sleep()

	check = bumperData()

	if check == True:
		for i in range(0,5):
			publish_PID_vel(0,0)
			rate.sleep()
		sleep(2)	
		for i in range(0,10):
			publish_PID_vel(-0.2, -0.2)
			rate.sleep()

		publish_PID_vel(0,0)



def rotateTo(angle):
	kp = 0.573
	ki = 0.0191
	kd = 0.42975

	integral = 0
	last_error = 0
	derivative = 0
	i = 0

	target_achieved = False

	target = angle
	while not target_achieved:

		check = bumperData()
		if check == True:
			break
		(trans,quat) = check_camera()
		euler = tf.transformations.euler_from_quaternion(quat)
		yaw = euler[2]
		error = target - yaw
		if error > radians(180):
			error = error - radians(360)
		if error < radians(-180):
			error = error + radians(360)
		print "Error: "+ str(error)
		integral += error
		if (error>0 and last_error<0) or (error<0 and last_error>0):
			intergal = 0
		derivative = error - last_error
		ang_vel = kp*error + ki*integral + kd*derivative
		ang_vel = ang_vel * 0.1175 #converting angular velocity to wheel speeds
		print "Ang_vel" + str(ang_vel)
		print "Target angle: " + str(degrees(target)) 
		if abs(error) < 0.01: #old threshold =0.006
			ang_vel = 0
			i += 1
		publish_PID_vel(ang_vel,-1*ang_vel)
		last_error = error
		print "---"
		if i >= 5:
			target_achieved = True
		rate.sleep()

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
