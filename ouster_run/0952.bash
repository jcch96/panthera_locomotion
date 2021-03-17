#!/bin/bash
cd && cd catkin_ws
source devel/setup.bash

roslaunch ouster_ros 0952.launch sensor_hostname:=169.254.63.172 udp_dest:=169.254.209.217 lidar_mode:=1024x20 viz:=false image:=false

