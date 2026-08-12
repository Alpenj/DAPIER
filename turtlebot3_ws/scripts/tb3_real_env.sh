#!/usr/bin/env bash
# ROS 2 environment for the physical TurtleBot3 Burger on the Jetson Nano.
# Source this file; do not execute it in a subshell.

source /opt/ros/jazzy/setup.bash
source "$HOME/DAPIER/turtlebot3_ws/install/setup.bash"

# ~/.bashrc intentionally isolates exam-time ROS nodes to localhost. Physical
# robot operation needs LAN discovery, so override that policy only here.
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_STATIC_PEERS

# Dedicated physical-robot domain. Domain 30 contained an unrelated
# TwistStamped /cmd_vel endpoint on the shared Wi-Fi.
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# The old static-peer XML disabled multicast and failed to discover the Nano.
# Default CycloneDDS multicast was verified against 192.168.0.253.
unset CYCLONEDDS_URI

export TURTLEBOT3_MODEL=burger
export TURTLEBOT3_NANO_IP=192.168.0.253
export TURTLEBOT3_NANO_USER=dapierttb
