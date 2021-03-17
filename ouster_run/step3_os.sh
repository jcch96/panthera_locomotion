#!/bin/bash

# Plug in lidar but do not turn on
sudo ip addr flush dev enx00051bc90121 # change ethernet name
sudo ip addr add 10.5.6.1/24 dev enx00051bc90121

echo 'Turn on Lidar and run ./step2_os.sh'

./step4_os.sh
