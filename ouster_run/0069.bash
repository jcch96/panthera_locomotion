#!/bin/bash
cd && cd catkin_ws
source devel/setup.bash

roslaunch ouster_ros 0069.launch sensor_hostname:=10.5.6.37 udp_dest:=10.5.6.1 lidar_mode:=1024x20 viz:=false image:=false
