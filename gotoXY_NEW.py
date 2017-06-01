#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
#from turtlesim.msg import Pose
from math import pow,atan2,sqrt
from nav_msgs.msg import Odometry

class gotoXY():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlesim_gotoXY', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.callback)
        #self.pose = Pose()
        self.pose = Odometry()
        self.rate = rospy.Rate(5)
        
    orientation = 0

    yaw = 0

    def convertQtoEuler():
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        
    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.pose.position.x = round(self.pose.pose.position.x, 4)
        self.pose.pose.position.y = round(self.pose.pose.position.y, 4)

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def move2goal(self):
        #goal_pose = Pose()
        goal_pose = Odometry()
        goal_pose.pose.position.x = input("Set your x goal: ")
        goal_pose.pose.position.y = input("Set your y goal: ")
	#goal_pose.theta = input("Set your final orientation: ")
        angle = input("Set your final orientation: ")
	#orientation = goal_pose.theta
        distance_tolerance = input("Set your tolerance: ")
        vel_msg = Twist()

        while sqrt(pow((goal_pose.pose.position.x - self.pose.pose.position.x), 2) + pow((goal_pose.pose.position.y - self.pose.pose.position.y), 2)) >= distance_tolerance:

            #Porportional Controller
            #linear velocity in the x-axis:
            vel_msg.linear.x = 1.5 * sqrt(pow((goal_pose.pose.position.x - self.pose.pose.position.x), 2) + pow((goal_pose.pose.position.y - self.pose.pose.position.y), 2))
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 4 * (atan2(goal_pose.pose.position.y - self.pose.pose.position.y, goal_pose.pose.position.x - self.pose.pose.position.x) - yaw)

            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z = orientation - yaw
        self.velocity_publisher.publish(vel_msg)

        rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        x = gotoXY()
        x.move2goal()

    except rospy.ROSInterruptException: pass

