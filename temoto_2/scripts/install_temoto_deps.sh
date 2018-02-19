#!/bin/bash

RESET="\e[39m\e[0m"

# Go back to the catkin_workspace/src folder
P1=$(awk -F: '{print $1}' <<< "$ROS_PACKAGE_PATH")
P2=$(awk -F: '{print $2}' <<< "$ROS_PACKAGE_PATH")
CW_DIR="Uninitialized directory"

# Find the catkin_workspace directory
if [[ $P1 = *"opt"* ]]; then
  CW_DIR=$P2
else
  CW_DIR=$P1
fi

# Save the currend directory and go to the catcin_ws/src directory
PREV_DIR=$(pwd)
cd $CW_DIR

# Clone the rviz_plugin_manager package
echo -e $RESET"\n\e[32mCloning the rviz_plugin_manager package to\e[1m" $CW_DIR $RESET
git clone https://github.com/UTNuclearRoboticsPublic/rviz_plugin_manager

# Clone the human_msgs package
echo -e $RESET"\n\e[32mCloning the human_msgs package to\e[1m" $CW_DIR $RESET
git clone https://github.com/ut-ims-robotics/human_msgs

cd $PREV_DIR

