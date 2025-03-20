#!/bin/bash

# Define workspace path
WS_PATH=~/husky_ws_3

echo "Creating workspace at $WS_PATH..."
mkdir -p $WS_PATH/src
cd $WS_PATH

# Initialize colcon workspace
echo "Building empty workspace..."
colcon build --symlink-install
source install/setup.bash

# Move to the src directory
cd $WS_PATH/src

# Clone repositories
echo "Cloning repositories..."
git clone https://github.com/clearpathrobotics/clearpath_nav2_demos.git
git clone https://github.com/SteveMacenski/slam_toolbox.git
git clone https://github.com/ros2/geometry2.git
git clone https://github.com/ros-planning/navigation2.git

# Checkout correct branches
echo "Checking out correct branches..."
cd $WS_PATH/src/clearpath_nav2_demos && git checkout jazzy
cd $WS_PATH/src/slam_toolbox && git checkout humble
cd $WS_PATH/src/geometry2 && git checkout humble
cd $WS_PATH/src/navigation2 && git checkout humble

echo "Setup complete!"

