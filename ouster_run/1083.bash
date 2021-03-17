#!/bin/bash
cd && cd catkin_ws
source devel/setup.bash

roslaunch ouster_ros ouster.launch sensor_hostname:=10.5.5.87 udp_dest:=10.5.5.1 lidar_mode:=1024x20 viz:=false image:=false
