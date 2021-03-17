#!/bin/bash

# Plug in lidar but do not turn on
sudo ip addr flush dev enx00051bc90130 # change ethernet name
sudo ip addr add 10.5.5.1/24 dev enx00051bc90130

#echo 'Turn on Lidar and run ./step2_os.sh'
./step2_os.sh
