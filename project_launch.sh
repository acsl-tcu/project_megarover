#! /usr/bin/bash +x
source $ACSL_ROS2_DIR/bashrc
echo $PROJECT
echo $ROS_DOMAIN_ID

cd $ACSL_ROS2_DIR/0_host_commands/scripts/
#dup microros
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB-megarover -v4 --ros-args --remap __node:=microros_node --remap __ns:=/${HOSTNAME}
