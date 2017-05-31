#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

rospy.init_node('joystick_control')

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rate = rospy.Rate(5)

def callback(data):
	cmd_msg = Twist()
	brake = data.buttons[7]
	ly = data.axes[1]
	rx = data.axes[2]
	x_vel = ly*3
	ang_vel = rx*3
	if (brake == 1):
		x_vel = 0
		ang_vel = 0
	cmd_msg.linear.x = x_vel
	cmd_msg.angular.z = ang_vel
	cmd_vel_pub.publish(cmd_msg)
	rate.sleep()

def listener():

	rospy.Subscriber('/joy', Joy, callback)
	rospy.spin()

if __name__ == '__main__':
    listener()
