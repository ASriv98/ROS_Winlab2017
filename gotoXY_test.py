#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
#from turtlesim.msg import Pose
from math import pow,atan2,sqrt
from nav_msgs.msg import Odometry
import tf
import math
from time import sleep
from math import radians, degrees


rospy.init_node('gotoXY_test')
listener = tf.TransformListener()
rate = rospy.Rate(10)
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pose_subscriber = rospy.Subscriber('/odom', Odometry)
pose = Odometry()
goal_pose = Odometry()
vel_msg = Twist()


def getCameraData():
	got_one = False
	while not got_one:
		try:
			(trans,rot) = listener.lookupTransform('/map_zero', '/hd_cam_new', rospy.Time(0))
			got_one = True
			rate.sleep()

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rate.sleep()
			continue

		# trans contains the x, y, and z components of the position of the camera with respect to map (units are in meters)
		
		x = trans[0]
		y = trans[1]
		z = trans[2]

		# rot is a quaternion containing the rotational components of the translation between the map and the camera
		# Since euler angles are somewhat easier to work with, we will convert to those:

		euler = tf.transformations.euler_from_quaternion(rot)
		
		# Lastly, since the default units are radians, we will convert to degrees since it is more intuitive
		roll = math.degrees(euler[0])
		pitch = math.degrees(euler[1])
		yaw = (euler[2])
	return (x,y,yaw)

def getYaw(self):
	quaternion = (self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y, self.pose.pose.pose.orientation.z, self.pose.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	yaw = angle2pi(euler[2])
	return yaw

	#Changes to 2pi plane
def angle2pi(angle):
	if (angle < 0):
		angle = angle + (2*math.pi)
	return angle

def get_goal():
	#print goal_pose
	global goal_pose
	x = input("Set your x goal: ")
	y = input("Set your y goal: ")
	angle = input("Set your angle: ")
	cur_pos = getCameraData()
	current_x = cur_pos[0]
	current_y = cur_pos[1]
	goal_pose.pose.pose.position.x = current_x + x
	goal_pose.pose.pose.position.y = current_y + y
	#orientation = input("Set your final orientation: ")
	distance_tolerance = input("Set your tolerance: ")
	angle_tolerance = input("Set your angle tolerance: ")
	vel_msg = Twist()

def rotateTo(angle):

	temp = getCameraData()
	print temp
	if (angle > 0):
		v_z  = 1.2
	if (angle <0):
		v_z = 1.2
	time = angle/v_z
	print time



	refresh_rate = time / 0.1 
	print refresh_rate

	sleep_time = time - (0.1*int(refresh_rate))
	print sleep_time

	i = 0
	while (i <= int(refresh_rate)-1):
		vel_msg = Twist()
		vel_msg.angular.z = v_z
		velocity_publisher.publish(vel_msg)
		sleep(0.1)
		i += 1
		print i

	sleep(sleep_time)
	#for i in range(0,10):
	vel_msg.angular.z = 0
	velocity_publisher.publish(vel_msg)
	sleep(1)
		#rate.sleep()

	newtemp = getCameraData()
	print newtemp

	#if ((abs(temp[2]) < abs(newtemp[2])):
	difference = abs(newtemp[2]) + abs(temp[2])
	#else:
		#difference = 
	print difference

	if (angle > 0):
		while ((difference - math.pi) >= 0):
			v_z = -1.2
			new_angle = difference - math.pi
			print new_angle
			new_time = new_angle/v_z
			new_refresh_rate = new_time / 0.1
			new_sleep_time = new_time - (0.1*int(new_refresh_rate))
			temp = 0

			while (temp <= (int(new_refresh_rate)-1)):
				vel_msg = Twist()
				vel_msg.angular.z = v_z
				velocity_publisher.publish(vel_msg)
				#print 'hello'
				sleep(0.1)
				temp+=1

			#sleep(new_sleep_time)
			for temp in range(0,10):
				vel_msg.angular.z = 0
				velocity_publisher.publish(vel_msg)

			sleep(2)
			newtemp = getCameraData()
			difference = abs(newtemp[2]) + abs(temp[2])

	if (angle < 0):
		while ((difference - math.pi) >= 0):
			v_z = 1.2
			new_angle = difference - math.pi
			new_time = new_angle/v_z
			new_refresh_rate = new_time / 0.1
			new_sleep_time = new_time - (0.1*int(new_refresh_rate))
			temp = 0
			while (temp <= int(new_refresh_rate)-1):
				vel_msg = Twist()
				vel_msg.angular.z = v_z
				velocity_publisher.publish(vel_msg)
				sleep(0.1)
				temp+=1

			#sleep(new_sleep_time)
			for temp in range(0,10):
				vel_msg.angular.z = 0
				velocity_publisher.publish(vel_msg)
			sleep(2)
			newtemp = getCameraData()
			difference = abs(newtemp[2]) + abs(temp[2])

def rotateTo_withOdometry(angle):
	temp = getCameraData()
	yaw = temp[2]
	print temp

	#1.2 wasn't the most accurate speed (0.8 had slightly increased accuracy)
	if (angle > 0):
		v_z  = 1.2
	if (angle <0):
		v_z = 1.2

	clockwise = False; 
	turn_amount = angle - yaw;




def rotateTo_withOdometryXY(x,y):
	direction = atan2()







"""	
		
	#Callback function implementing the pose value received
	def callback(self, data):
		self.pose = data
		self.pose.pose.pose.position.x = round(self.pose.pose.pose.position.x, 4)
		self.pose.pose.pose.position.y = round(self.pose.pose.pose.position.y, 4)
	yaw = getYaw(self)

	def move2goal(self):
		#goal_pose = Pose()
		goal_pose = Odometry()
	#print goal_pose
		goal_pose.pose.pose.position.x = input("Set your x goal: ")
		goal_pose.pose.pose.position.y = input("Set your y goal: ")
		cur_pos = getCameraData()
		current_x = cur_pos[0]
		current_y = cur_pos[1]
		#orientation = input("Set your final orientation: ")
		distance_tolerance = input("Set your tolerance: ")
		angle_tolerance = input("Set your angle tolerance: ")
		vel_msg = Twist()
	
	yaw = getYaw(self)
	direction = atan2(goal_pose.pose.pose.position.y-self.pose.pose.pose.position.y, goal_pose.pose.pose.position.x-self.pose.pose.pose.position.x)
	direction = angle2pi(direction)
	print "direction:" 
	print direction

	# sets the direction of turn
	clockwise = False
	if (direction - yaw) <= math.pi:
		if abs(direction - yaw) > 0:
			clockwise = False
		else:
			clockwise = True
	if (direction - yaw) > math.pi:
		if abs(direction - yaw) > 0:
			clockwise = True
		else:
			clockwise = False

	#Turns the robot   
		while abs(yaw - direction) >= angle_tolerance:
			#angular velocity in the z-axis:
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			v_z = 4 * (direction - yaw)
		
		if abs(v_z) > 1:
			v_z = 1
		if clockwise == True:
			v_z = -abs(v_z)
		else:
			v_z = abs(v_z)

		vel_msg.angular.z = v_z 
		print "Yaw:" 
		print yaw
		yaw = getYaw(self)
	
		self.velocity_publisher.publish(vel_msg)
		self.rate.sleep()
	print yaw
	vel_msg.angular.z = 0
	self.velocity_publisher.publish(vel_msg)
	self.rate.sleep()
		
	target_distance  = math.sqrt((goal_pose.pose.pose.position.x-self.pose.pose.pose.position.x)**2 + (goal_pose.pose.pose.position.y-self.pose.pose.pose.position.y)**2)
	current_distance = 0

	while (target_distance - current_distance >= distance_tolerance):
		
		#Proportional Controller
		#linear velocity in the x-axis:

		v_x = 1.5 * sqrt(pow((goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x), 2) + pow((goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y), 2))
		if(v_x > 0.5):
			v_x = 0.5
		vel_msg.linear.x = v_x
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		print "target_distance:"
		print target_distance
		print "current_distance:"
		print current_distance
		#target_distance  = math.sqrt((goal_pose.pose.pose.position.x)**2 + (goal_pose.pose.pose.position.y)**2)
		current_distance = math.sqrt((self.pose.pose.pose.position.x-current_x)**2 + (self.pose.pose.pose.position.y-current_y)**2)

		#Publishing our vel_msg
		self.velocity_publisher.publish(vel_msg)
		self.rate.sleep()
	#Stopping our robot after the movement is over
	vel_msg.linear.x = 0
	#vel_msg.angular.z = orientation - yaw
	#(temporary for testing purposes)
	# vel_msg.angular.z = 0
	self.velocity_publisher.publish(vel_msg)

		#rospy.spin()
"""
test = getCameraData()
print test
rotateTo(3.14)
sleep(2)
test = getCameraData()
print test
"""
if __name__ == '__main__':

	while not rospy.is_shutdown():

		#Testing our function
		#rotateTo(3.14)
		test = getCameraData()
		print test
		#x = gotoXY()
		#x.move2goal()
"""