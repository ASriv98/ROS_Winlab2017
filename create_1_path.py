#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
import csv
from geometry_msgs.msg import PoseStamped

rospy.init_node("create_1_path")

plan_publisher = rospy.Publisher('/create_1_waypoints', Path, queue_size=10)

rate = rospy.Rate(10.0)



def publish_plan():

	new_plan = Path()	

	new_plan.header.frame_id = 'map'

	with open('way_points.csv') as csvfile:
		points_reader = csv.reader(csvfile, delimiter = ',')
		for row in points_reader:
			point = PoseStamped()
			point.pose.position.x = float(row[0])
			point.pose.position.y = float(row[1])
			new_plan.poses.append(point)

	print "Publishing a new plan..."
	plan_publisher.publish(new_plan)
	print new_plan
while not rospy.is_shutdown():
	raw_input("Press enter to continue: ")
	publish_plan()
	