#!/bin/bash

. /opt/ros/kinetic/setup.bash
. /home/husarion/ros_workspace/devel/setup.sh

export ROS_MASTER_URI=http://master:11311
export ROS_IPV6=on

roslaunch rosbot_webui demo.launch