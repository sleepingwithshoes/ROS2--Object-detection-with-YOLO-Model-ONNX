#!/bin/zsh
set -e

# Install project-specific dependencies
echo "Installing project-specific dependencies..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y

source "/opt/ros/$ROS_DISTRO/setup.zsh"

# Build workspace
echo ""
echo "Building the ROS 2 workspace..."
if [ -d "build" ]; then
    echo "Removing existing build directory..."
    rm -rf build
fi
if [ -d "install" ]; then
    echo "Removing existing install directory..."
    rm -rf install
fi
if [ -d "log" ]; then
    echo "Removing existing log directory..."
    rm -rf log
fi

colcon build --symlink-install

# Source workspace if built
if [ -f "install/setup.zsh" ]; then
    source "install/setup.zsh"
fi

exec zsh