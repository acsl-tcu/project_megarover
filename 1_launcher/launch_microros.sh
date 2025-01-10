#! /usr/bin/bash

$(echo "exec ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB-megarover --log-opt max-size=100m --log-opt max-file=10 --ros-args --remap __node:=microros_node --remap __ns:=/${HOSTNAME} -v6 --namespace-remapping /${HOSTNAME}")
