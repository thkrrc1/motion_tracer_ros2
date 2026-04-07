#!/bin/bash
##----- version check ----------
version=`cat /etc/lsb-release | grep DISTRIB_RELEASE`
if [[ ${version} =~ "16" ]]; then
  ros_ws=/home/seed/ros/kinetic
elif [[ ${version} =~ "18" ]]; then
  ros_ws=/home/seed/ros/melodic
elif [[ ${version} =~ "20" ]]; then
  ros_ws=/home/seed/ros/noetic
fi
##-------------------------------
source ${ros_ws}/devel/setup.bash

gnome-terminal --zoom=0.5 --geometry=+0+0 --tab -e 'bash -c "roslaunch motion_tracer robot_bringup.launch DUALSHOCK:=true "'
