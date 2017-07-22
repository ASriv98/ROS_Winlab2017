** First create a file under /etc/systemd/system/ and call it robot_launch.service

sudo nano /etc/systemd/system/robot_launch.service

** Make it look like this:

[Unit]
Description=Robot Launching Servive <see /usr/local/bin/robot_launch.sh>
Requires=network-online.target
Wants=network-online.target
After=network-online.target

[Service]
ExecStart=/usr/local/bin/robot_launch.sh
ExecStop=/usr/local/bin/robot_launch.sh -1
StandardOutput=console
KillMode=process

[Install]
WantedBy=multi-user.target

** Next, go to /usr/local/bin/ and create robot_launch.sh, and give it executable permissions

sudo nano /usr/local/bin/robot_launch.sh
sudo chmod +x robot_launch.sh

** Make it look like this:

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

** Next we need to make sure that systemd-networkd-wait-online.service is enabled, otherwis chrony won’t update properly. Also enable the service we just created

sudo systemctl enable systemd-networkd-wait-online.service
sudo systemctl enable robot_launch.service

** And that’s all folks, easy right? I’m sure that didn’t take someone a whole day to figure out. To check on your service one the pi has booted up, do:

sudo systemctl status robot_launch

** Now, to shut down your service, killing all of the ros nodes, use the following command:

sudo systemctl stop robot_launch
