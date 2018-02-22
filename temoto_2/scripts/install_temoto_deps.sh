#!/bin/bash

RED="\e[31m"
YELLOW="\e[33m"
GREEN="\e[32m"
BOLD="\e[1m"
NL="\n"
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

if [[ -z $CW_DIR ]]; then
  echo -e $RED$BOLD"Could not find the catkin workspace, have you sourced it? Exiting."$RESET
  exit
fi

# Save the currend directory and go to the catcin_ws/src directory
PREV_DIR=$(pwd)
cd $CW_DIR

# Check if the rviz_plugin_manager package exists
rospack find rviz_plugin_manager &> /dev/null

if [[ $? = 0 ]]; then
  echo -e $GREEN$BOLD"* rviz_plugin_manager" $RESET$GREEN"package is already installed."$RESET
else
  # Clone the rviz_plugin_manager package
  echo -e $RESET$GREEN"Cloning the rviz_plugin_manager package to"$BOLD $CW_DIR $RESET
  git clone https://github.com/UTNuclearRoboticsPublic/rviz_plugin_manager
fi

# Check if the human_msgs package exists
rospack find human_msgs &> /dev/null

if [[ $? = 0 ]]; then
  echo -e $GREEN$BOLD"* human_msgs" $RESET$GREEN"package is already installed."$RESET
else
  # Clone the rviz_plugin_manager package
  echo -e $RESET$GREEN"Cloning the human_msgs package to"$BOLD $CW_DIR $RESET
  git clone https://github.com/ut-ims-robotics/human_msgs
fi

cd $PREV_DIR

echo -e $NL"Dependencies are installed, you are good to go."
