#!/bin/bash
ulimit -c unlimited
source /opt/ros/jazzy/setup.bash
source ~/ros2/jazzy/install/setup.bash
cd ~/
sleep 1
gnome-terminal --tab -e 'bash -c "ulimit -c unlimited; ros2 launch motion_tracer_ros2 tracer_bringup.launch.py"'
