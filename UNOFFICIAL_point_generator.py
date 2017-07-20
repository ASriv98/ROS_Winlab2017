#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
#from turtlesim.msg import Pose
from math import pow,atan2,sqrt
from nav_msgs.msg import Odometry
import tf
from math import radians, degrees, sqrt
from time import sleep

max_dist = 0.5

target_x = input("Set your x: ")  
target_y = input("Set your y: ")

points_list = []

def list_generator():
    new_x = current_x
    new_y = current_y
    while(new_x != target_y and new_y != target_y):
        if target_x > new_x:
            new_x += 0.5
        else if target_x < new_x:
            new_x -= 0.5
        else if abs(target_x - current_x) < 0.5:
            new_x = target_x
        else:
            new_x = new_x
            
        if target_y > new_y:
            new_y += 0.5
        else if: target_y < new_y:
            new_y -= 0.5
        else if abs(target_y - current_y) < 0.5:
            new_x = target_x
        else:
            new_y = new_y

        new_point = (new_x, new_y)

        points_list.append(new_point)

    
        
