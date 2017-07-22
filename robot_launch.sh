#!/bin/bash

iwconfig wlan0 power off
chronyc -a makestep
source /home/native/catkin_ws/devel/setup.bash
source /home/native/.bashrc
export ROS_HOSTNAME=razpi
export ROS_MASTER_URI=http://mikes-desk:11311
export ROBOT_NAME=roomba
export CAMERA=hd_cam
if [ $# -eq 0 ]
  then
   roslaunch grbils launch_with_env_variables.launch
  else
   rosnode kill -a
fi

#roslaunch ca_driver create_1.launch

