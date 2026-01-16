#!/bin/bash

# Ask user for robot number
echo "Enter robot number (1–5):"
read ROBOT_NUM

# Base IP prefix
BASE_IP="192.168.1.10"

# Check valid input
if [[ "$ROBOT_NUM" =~ ^[1-5]$ ]]; then
    IP="${BASE_IP}${ROBOT_NUM}"
    echo "Starting Zenoh bridge for Robot $ROBOT_NUM at $IP:7447"
    zenoh-bridge-ros2dds -e tcp/${IP}:7447
else
    echo "❌ Invalid robot number. Please enter a number between 1 and 5."
    exit 1
fi
