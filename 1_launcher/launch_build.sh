#! /usr/bin/bash
source /common/scripts/super_echo
becho "colcon build" $@
cd /root/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
if [ $# -ge 1 ]; then
  colcon build --symlink-install --packages-select $@ --parallel-workers $(($(nproc) - 1))
else
  colcon build --symlink-install --parallel-workers $(($(nproc) - 1))
fi
source install/setup.bash
