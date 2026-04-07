#!/bin/bash
ros_ws=$1
robot_ip=$2
user_name=$3
password=$4
local_ip=$5
command=$6

##----- version check ----------
#version=`cat /etc/lsb-release | grep DISTRIB_RELEASE`
#if [[ ${version} =~ "16" ]]; then
#  ros_ws=/home/seed/ros/kinetic
#elif [[ ${version} =~ "18" ]]; then
#  ros_ws=/home/seed/ros/melodic
#elif [[ ${version} =~ "20" ]]; then
#  ros_ws=/home/seed/ros/noetic
#fi
##-------------------------------
source ${ros_ws}/devel/setup.bash
roscd motion_tracer/scripts

if [ ${command} = "controller" ]; then
  dual_shock=$7

  expect ssh.exp ${robot_ip} ${password} "rosnode kill --all & killall -9 roscore & killall -9 rosmaster;killall gnome-terminal-server;exit";
  gnome-terminal --zoom=0.5 \
    --tab -e 'bash -c "expect ssh.exp '${robot_ip}' '${password}' \"export ROS_IP='${robot_ip}'\" \"sleep 1;roslaunch motion_tracer robot_bringup.launch DUALSHOCK:='${dual_shock}' \" "';

elif [ ${command} = "tracer" ]; then
  device=$7
  dual_shock=$8
  initial_pose=$9
  neck_movement=${10}
  neck_offset=${11}
  neck_auto=${12}
  neck_reverse=${13}

  export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
  if [ ${device} = "mechaless" ]; then
    gnome-terminal --tab -e 'bash -c "sleep 3; export LD_LIBRARY_PATH=/usr/local/lib/nuitrack:$LD_LIBRARY_PATH; source '${ros_ws}'/devel/setup.bash; roslaunch nuitrack_body_tracker nuitrack_body_tracker.launch"' \
     --tab -e 'bash -c "sleep 5; source '${ros_ws}'/devel/setup.bash; roslaunch skeleton_processor skeleton_sending_angle.launch"' \
     --tab -e 'bash -c "sleep 10; source /opt/intel/openvino/bin/setupvars.sh; source '${ros_ws}'/devel/setup.bash; roslaunch skeleton_processor skeleton_drawing.launch"' \
     --tab -e 'bash -c "sleep 12; echo $LD_LIBRARY_PATH; source '${ros_ws}'/devel/setup.bash; source /opt/intel/openvino/bin/setupvars.sh; sleep 5; roslaunch skeleton_processor face_detection.launch"' \
     --tab -e 'bash -c "sleep 14; source '${ros_ws}'/devel/setup.bash; rosrun skeleton_processor skeleton_processor_pygame.py"';

  elif [ ${device} = "mechanical" ]; then
    gnome-terminal --zoom=0.5 --tab -e 'bash -c "roslaunch --wait motion_tracer tracer_bringup.launch \\
      DUALSHOCK:='${dual_shock}' initial_pose:='${initial_pose}' neck_movement:='${neck_movement}' neck_offset:='${neck_offset}' neck_auto:='${neck_auto}' neck_reverse:='${neck_reverse}' "';
  fi

elif [ ${command} = "rviz" ]; then
  export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "roslaunch motion_tracer view.launch"'

elif [ ${command} = "path" ]; then
  export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
  echo 'ROS_MASTER_URI is ' $ROS_MASTER_URI
  echo 'ROS_IP is ' $ROS_IP

elif [ ${command} = "record" ]; then
  argument=$7
  file_sufix=$8
  if [ ${argument} = "start" ]; then
    export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
    gnome-terminal --zoom=0.5 --tab -e 'bash -c "roslaunch motion_tracer recording.launch file_sufix:='${file_sufix}';exit "'

  elif [ ${argument} = "stop" ]; then
    export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
    rosnode kill tracer_recording_node&
    rosnode kill joy_recording_node
  fi

elif [ ${command} = "play" ]; then
  file_sufix=$7
  export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "roslaunch motion_tracer playback.launch file_sufix:='${file_sufix}';exit "'

elif [ ${command} = "initial_pose" ]; then
  initial_pose=$7
  export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
  rosrun dynamic_reconfigure dynparam set /tracer_teleop_node 'initial_pose' ${initial_pose}
  rosparam set /initial_pose ${initial_pose}

elif [ ${command} = "neck_pose" ]; then
  neck_movement=$7
  neck_offset=$8
  neck_auto=$9
  neck_reverse=${10}


  export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
  rosrun dynamic_reconfigure dynparam set upper_controller_node \
    "{'movement':'${neck_movement}', 'automatically':${neck_auto}, 'reverse':${neck_reverse}, 'offset':${neck_offset}}" 

elif [ ${command} = "kill" ]; then
  remote_ros_ws=$7
  rosnode kill --all & killall -9 roscore & killall -9 rosmaster;
  killall gnome-terminal-server;
  expect ssh.exp ${user_name}@${robot_ip} ${password} "export DISPLAY=:0.0;killall gnome-terminal-server; bash '${remote_ros_ws}'/src/motion_tracer/scripts/teleop_mover.sh"
fi
