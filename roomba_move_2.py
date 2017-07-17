#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
#from turtlesim.msg import Pose
from math import pow,atan2,sqrt
from nav_msgs.msg import Odometry
import tf
from math import radians, degrees, sqrt
from time import sleep


rospy.init_node("roomba_pid_controller")

velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pose_pub = rospy.Publisher('/start_pose', Twist, queue_size=10)

rate = rospy.Rate(10.0)

listener = tf.TransformListener()

sample_points = []


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

def publish_cmd_vel(lin_vel, angle_vel):
	vel_msg = Twist()
	vel_msg.linear.x = lin_vel
	vel_msg.angular.z = angle_vel
	velocity_publisher.publish(vel_msg)

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

def rotateTo(angle):
	kp = 0.955
	ki = 0
	kd = 0

	integral = 0
	last_error = 0
	derivative = 0
	i = 0

	target_achieved = False

	target = angle
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
		print "Target angle: " + str(degrees(target)) 
		if abs(error) < 0.006:
			ang_vel = 0
			i += 1
		publish_cmd_vel(0,ang_vel)
		last_error = error
		print "---"
		if i >= 5:
			target_achieved = True
		rate.sleep()

def move2goal(sample_x, sample_y):
	(trans, rot) = check_camera()
	current_x = trans[0]
	current_y = trans[1]

	#target_x = input("Set your x: ")  
	#target_y = input("Set your y: ")

	target_x = sample_x
	target_y = sample_y

	direction = atan2((target_y-current_y),(target_x-current_x))
	print degrees(direction)

	rotateTo(direction)
	sleep(2)
	
	(trans, rot) = check_camera()
	current_x = trans[0]
	current_y = trans[1]
	movement_distance = sqrt((target_x-current_x)**2+(target_y-current_y)**2)
	moveTo(movement_distance)


while not rospy.is_shutdown():
	print check_camera()
	#move2goal()

	for point in sample_points:
                move2goal(point[0], point[1])

                
