#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
#from turtlesim.msg import Pose
from math import pow,atan2,sqrt
from nav_msgs.msg import Odometry
import tf
import math
from math import radians, degrees
from time import sleep

listener = tf.TransformListener()

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

def getCameraData():
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

		# rot is a quaternion containing the rotational components of the translation between the map and the camera
		# Since euler angles are somewhat easier to work with, we will convert to those:

    	euler = tf.transformations.euler_from_quaternion(rot)
		
		# Lastly, since the default units are radians, we will convert to degrees since it is more intuitive
    	roll = math.degrees(euler[0])
    	pitch = math.degrees(euler[1])
    	yaw = (euler[2])
        yaw = angle2pi(yaw)

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
    	camera_check = False
    xyYawTuple = (x,y,yaw)
    return xyYawTuple

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
	yaw = getYaw(self)

    def move2goal(self):
	
	#goal_pose = Pose()
        goal_pose = Odometry()
        #print goal_pose
        #goal_pose.pose.pose.position.x = input("Set your x goal: ")
      #  goal_pose.pose.pose.position.y = input("Set your y goal: ")
        cur_pos = getCameraData()
        current_x = cur_pos[0]
        current_y = cur_pos[1]
        yaw_camera= cur_pos[2]
        yaw_camera = angle2pi(yaw_camera)
            #orientation = input("Set your final orientation: ")
      #  distance_tolerance = input("Set your tolerance: ")
        angle_abs = input("Set your angle: ")
        angle_tolerance = input("Set your angle tolerance: ")
        print angle_abs
        vel_msg = Twist()
        
        yaw_odometry = getYaw(self)
        print "yaw_odom:"
        print yaw_odometry
        print "yaw camera"
        print yaw_camera

        #direction = atan2(goal_pose.pose.pose.position.y-self.pose.pose.pose.position.y, goal_pose.pose.pose.position.x-self.pose.pose.pose.position.x)
        #direction = angle2pi(direction)
       # print "direction:" 
        #print direction

        difference = angle_abs - yaw_camera
        print "difference"
        print difference
        angle_user = yaw_odometry + difference
        print "angle"
        print angle_user
        angle_user = angle2pi(angle_user)

        angle_tolerance = 0.1
        # sets the direction of turn
        clockwise = False
        if abs(angle_user - yaw_camera) <= math.pi:
            if angle_user - yaw_camera > 0:
                clockwise = True
            else:
                clockwise = False
        if abs(angle_user - yaw_camera) > math.pi:
            if angle_user - yaw_camera > 0:
                clockwise = False
            else:
                clockwise = True

        #Turns the robot   
        while abs(yaw_odometry - angle_user) >= angle_tolerance:
                #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            v_z = 4 * (angle_user - yaw_odometry)
            
            if abs(v_z) > 1:
                v_z = 1
            if clockwise == True:
                v_z = -abs(v_z)
            else:
                v_z = abs(v_z)

            vel_msg.angular.z = v_z 
            print "Yaw_Odometry:" 
            print yaw_odometry
            print "angle:"
            print angle_user
            yaw_odometry = getYaw(self)
        
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        print yaw_odometry
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        sleep(1)

        cur_pos = getCameraData()
        yaw_camera = cur_pos[2]
        yaw_camera = angle2pi(yaw_camera)
        print yaw_camera
        sleep(1)


        while abs(angle_abs - yaw_camera) > 0.05:
            new_turn = angle_abs - yaw_camera
            angle_user = yaw_odometry + new_turn
            angle_user = angle2pi(angle_user)

            """clockwise = False
            if abs(angle_user - yaw_camera) <= math.pi:
                if angle_user - yaw_camera > 0:
                    clockwise = True
                else:
                    clockwise = False
            if abs(angle_user - yaw_camera) > math.pi:
                if angle_user - yaw_camera > 0:
                    clockwise = False
                else:
                    clockwise = True"""

            if angle_user > yaw_camera:
                clockwise = False
            else: 
                clockwise = True


            while abs(yaw_odometry - angle_user) >= angle_tolerance:
                #angular velocity in the z-axis:
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                v_z = 4 * (angle_user - yaw_odometry)
            
                if abs(v_z) > 1:
                    v_z = 0.5
                if clockwise == True:
                    v_z = -abs(v_z)
                else:
                    v_z = abs(v_z)

                vel_msg.angular.z = v_z 
                print "Yaw_Odometry:" 
                print yaw_odometry
                print "angle:"
                print angle_user
                yaw_odometry = getYaw(self)
        
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()
            print yaw_odometry
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            sleep(1)

            cur_pos = getCameraData()
            yaw_camera = cur_pos[2]
            yaw_camera = angle2pi(yaw_camera)
            print yaw_camera
            sleep(1)  



        


    	"""target_distance  = math.sqrt((goal_pose.pose.pose.position.x-current_x)**2 + (goal_pose.pose.pose.position.y-current_y)**2)
    	current_distance = 0

	    while (target_distance - current_distance >= distance_tolerance):
		
            #Proportional Controller
            #linear velocity in the x-axis:

            v_x = 1.5 * sqrt(pow((goal_pose.pose.pose.position.x - current_x), 2) + pow((goal_pose.pose.pose.position.y - current_y), 2))
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
	    self_data = getCameraData()
	    current_distance = math.sqrt((self_data[0]-current_x)**2 + (self_data[1]-current_y)**2)

            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        #vel_msg.angular.z = orientation - yaw
	#(temporary for testing purposes)
	# vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)"""

        rospy.spin()

if __name__ == '__main__':

    while not rospy.is_shutdown():

        #Testing our function
	
        x = gotoXY()
        x.move2goal()

