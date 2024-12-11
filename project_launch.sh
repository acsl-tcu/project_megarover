#! /usr/bin/bash +x

source $ACSL_ROS2_DIR/bashrc
echo $PROJECT
echo $ROS_DOMAIN_ID

cd $ACSL_ROS2_DIR/0_host_commands/scripts
#dup rf_robot
TAG=rplidar CONTAINER_NAME=rplidar_front COMPOSE_PROJECT_NAME=rplidar_front_rf ROS_LAUNCH=launch_rplidar.sh LARGS=front HOSTNAME=$(hostname | sed -e 's/-/_/g') docker compose -f $ACSL_ROS2_DIR/4_docker/docker-compose.yml up common -d
TAG=rplidar CONTAINER_NAME=rplidar_behind COMPOSE_PROJECT_NAME=rplidar_behind_rf ROS_LAUNCH=launch_rplidar.sh LARGS=behind HOSTNAME=$(hostname | sed -e 's/-/_/g') docker compose -f $ACSL_ROS2_DIR/4_docker/docker-compose.yml up common -d
dup microros
