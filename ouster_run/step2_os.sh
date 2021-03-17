#!/bin/bash

# Turn on Lidar
sudo ip link set enx00051bc90130 up
sudo dnsmasq -C /dev/null -kd -F 10.5.5.50,10.5.5.100 -i enx00051bc90130 --bind-dynamic

echo 'Open new terminal and run roslaunch file'


