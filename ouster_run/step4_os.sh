#!/bin/bash

# Turn on Lidar
sudo ip link set enx00051bc90121 up
sudo dnsmasq -C /dev/null -kd -F 10.5.6.1,10.5.6.254 -i enx00051bc90121 --bind-dynamic

echo 'Open new terminal and run roslaunch file'


