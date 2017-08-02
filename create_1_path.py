#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
import csv
from geometry_msgs.msg import PoseStamped

rospy.init_node("create_1_path")

plan_publisher = rospy.Publisher('new_plan', Path, queue_size=10)

rate = rospy.Rate(10.0)

def input_points():
        with open('points.csv') as csvfile:
                points_reader = csv.reader(csvfile, delimiter = ',')
                for row in points_reader:

def publish_plan():

	new_plan = Path()	

	new_plan.header.frame_id = 'map'

		with open('points.csv') as csvfile:
			points_reader = csv.reader(csvfile, delimiter = ',')
			for row in points_reader:
				point = PoseStamped()
				point.pose.position.x = row[0]
				point.pose.position.y = row[1]
				new_plan.poses.append(point)

	print "Publishing a new plan..."
	plan_publisher.publish(new_plan)

publish_plan()
	