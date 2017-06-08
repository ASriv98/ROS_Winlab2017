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
        self.rate = rospy.Rate(5)
        
    orientation = 0

        
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
	#goal_pose    print vel_msgse.theta = input("Set your final orientation: ")
        orientation = input("Set your final orientation: ")
        distance_tolerance = input("Set your tolerance: ")
        angle_tolerance = input("Set your angle tolerance: ")
        vel_msg = Twist()

	quaternion = (self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y, self.pose.pose.pose.orientation.z, self.pose.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	yaw = euler[2]
	angle = radians(yaw)
        while abs(atan2(goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y, goal_pose.pose.pose.position.x-self.pose.pose.pose.position.x) - angle) >= angle_tolerance:
            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            v_z = 4 * (atan2(goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y, goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x) - angle)
            if(v_z < -0.75):
                 v_z = -0.75
            if(v_z > 0.75):
                 v_z = 0.75
	    vel_msg.angular.z = v_z 
	    print angle
	    quaternion = (self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y, self.pose.pose.pose.orientation.z, self.pose.pose.pose.orientation.w)
	    euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]
	    angle = radians(yaw)
	print angle

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

	vel_msg.angular.z = 0
	self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()
        
        vel_msg.angular.z = orientation - yaw
	#(temporary for testing purposes)
	# vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        rospy.spin()

if __name__ == '__main__':

    while not rospy.is_shutdown():

        #Testing our function
	
        x = gotoXY()
        x.move2goal()

