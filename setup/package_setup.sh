#!/bin/bash

cd $WS_PATH/src

rosdep update
rosdep install --from-paths src --ignore-src -r -y 


colcon build --symlink-install 

source $WS_PATH/install/setup.bash


echo "source $WS_PATH/install/setup.bash" >> ~/.bashrc  
source ~/.bashrc