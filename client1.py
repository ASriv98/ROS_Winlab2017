#!/usr/bin/env python

import sys
import rospy
from irobot_create_2_1.srv import Brake

def brake_service():
    rospy.wait_for_service('/brake')
    try:
        brake_service = rospy.ServiceProxy('/brake', Brake)
        resp1 = brake_service(true)
        print(resp1.success)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
        brake_service()
