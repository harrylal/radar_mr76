#!/bin/bash 
sudo slcand -o -c -f -s6 /dev/ttyUSB0 slcan0

#115200
sudo ifconfig slcan0 up

rosrun socketcan_bridge socketcan_to_topic_node _can_device:=slcan0 &
echo "Socketcan Bridge CONNECTED"

