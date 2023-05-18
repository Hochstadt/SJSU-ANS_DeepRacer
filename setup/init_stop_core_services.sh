#!/usr/bin/env bash

source /opt/ros/foxy/setup.bash

# If Deepracer core service is running, stop it
systemctl is-active --quiet deepracer-core.service && systemctl stop deepracer-core.service

# Stop the existing daemon
ros2 daemon stop
