#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

exec "$@"
