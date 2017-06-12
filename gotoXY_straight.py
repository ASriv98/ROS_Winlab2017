#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
#from turtlesim.msg import Pose
from math import pow,atan2,sqrt
from nav_msgs.msg import Odometry
import tf
import math
from math import radians, degrees

class gotoXY():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('gotoXY_straight')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.callback)
        #self.pose = Pose()
        self.pose = Odometry()
        self.rate = rospy.Rate(10)


        
    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.pose.pose.position.x = round(self.pose.pose.pose.position.x, 4)
        self.pose.pose.pose.position.y = round(self.pose.pose.pose.position.y, 4)
	quaternion = (self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y, self.pose.pose.pose.orientation.z, self.pose.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
 	roll = euler[0]
        pitch = euler[1]
        global yaw
	yaw = euler[2]


    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.position.x), 2) + pow((goal_y - self.pose.position.y), 2))
        return distance

    def move2goal(self):
        #goal_pose = Pose()
        goal_pose = Odometry()
	#print goal_pose
        goal_pose.pose.pose.position.x = input("Set your x goal: ")
        goal_pose.pose.pose.position.y = input("Set your y goal: ")
	current_x = self.pose.pose.pose.position.x
	current_y = self.pose.pose.pose.position.y
        #orientation = input("Set your final orientation: ")
        distance_tolerance = input("Set your tolerance: ")
        angle_tolerance = input("Set your angle tolerance: ")
        vel_msg = Twist()

	quaternion = (self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y, self.pose.pose.pose.orientation.z, self.pose.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	yaw = euler[2]
	
	#Changes to 2pi plane
	if (yaw < 0):
	    yaw = yaw + (2*math.pi)
	direction = atan2(goal_pose.pose.pose.position.y-self.pose.pose.pose.position.y, goal_pose.pose.pose.position.x-self.pose.pose.pose.position.x)
	if (direction < 0):
	    direction = direction + (2*math.pi)
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
	    quaternion = (self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y, self.pose.pose.pose.orientation.z, self.pose.pose.pose.orientation.w)
	    euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]
	    if (yaw < 0):
	        yaw = yaw + (2*math.pi)
	   
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
	print yaw
	vel_msg.angular.z = 0
	self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()
        
	target_distance  = math.sqrt((goal_pose.pose.pose.position.x-self.pose.pose.pose.position.x)**2 + (goal_pose.pose.pose.position.y-self.pose.pose.pose.position.y)**2)
	current_distance = 0

	'''
	while sqrt(pow((goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x), 2) + pow((goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y), 2)) >= distance_tolerance:'''
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

        rospy.spin()

if __name__ == '__main__':

    while not rospy.is_shutdown():

        #Testing our function
	
        x = gotoXY()
        x.move2goal()

