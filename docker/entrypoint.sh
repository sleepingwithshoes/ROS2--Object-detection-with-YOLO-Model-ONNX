#!/bin/zsh
set -e

# Source ROS environment
source "/opt/ros/$ROS_DISTRO/setup.zsh"

# Source workspace if built
if [ -f "/workspace/install/setup.zsh" ]; then
    source "/workspace/install/setup.zsh"
fi

exec zsh