#!/bin/bash

source /home/jetson/scarlet/assets/usb-setup.sh
sudo chmod 666 /dev/ttyUSB0
export ROS_IP=scarlet.local
export ROS_MASTER_URI=http://scarlet.local:11311
