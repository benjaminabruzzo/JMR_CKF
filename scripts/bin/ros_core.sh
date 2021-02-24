#!/bin/bash
# file: core.sh

source /opt/ros/kinetic/setup.bash
export ROS_PACKAGE_PATH=~/ros:$ROS_PACKAGE_PATH

roscore
