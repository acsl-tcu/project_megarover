#! /usr/bin/bash

source /root/.venv/bin/activate
cp -r /common/ros_launcher/Arduino/* /root/Arduino/
/root/arduino-ide_2.3.4_Linux_64bit.AppImage --no-sandbox
