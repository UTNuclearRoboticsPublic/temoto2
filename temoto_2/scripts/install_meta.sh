#!/bin/bash

RED="\e[31m"
YELLOW="\e[33m"
GREEN="\e[32m"
BOLD="\e[1m"
NL="\n"
RESET="\e[39m\e[0m"

# Find the catkin_workspace directory
P1=$(awk -F: '{print $1}' <<< "$ROS_PACKAGE_PATH")
P2=$(awk -F: '{print $2}' <<< "$ROS_PACKAGE_PATH")
CW_DIR="Uninitialized directory"

if [[ $P1 = *"opt"* ]]; then
  CW_DIR=$P2
else
  CW_DIR=$P1
fi

TEMOTO_DIR=$CW_DIR/temoto2/temoto_2

if [ ! -d "$TEMOTO_DIR" ]; then
  echo -e $RED"Could not find the temoto_2 folder. Looked at '"$TEMOTO_DIR"'"$RESET
  exit
fi

META_SRC_DIR=$TEMOTO_DIR/language_processors/meta/meta_nlp

# Update and install the dependancies
echo -e $YELLOW$NL"Insert your sudo password to update and install the dependencies (g++, git, cmake, make, libjemalloc-dev, zlib1g-dev, wget)"$RESET
sudo apt update
sudo apt install g++ git cmake make libjemalloc-dev zlib1g-dev wget


# # # # # # # #
# Install MeTA
# # # # # # # #

PREV_DIR=$(pwd)
mkdir -p $META_SRC_DIR
cd $META_SRC_DIR

# Clone the MeTA repository
echo -e $RESET $GREEN $NL"Cloning the MeTA repository to"$BOLD $META_SRC_DIR $RESET
git clone https://github.com/meta-toolkit/meta.git ./
git submodule update --init --recursive

# Build MeTA
echo -e $RESET $GREEN $NL"Building MeTA ..." $RESET
mkdir $META_SRC_DIR/build
cp $META_SRC_DIR/config.toml $META_SRC_DIR/build/config.toml
cd $META_SRC_DIR/build
cmake ../ -DCMAKE_BUILD_TYPE=Release
make -j8

# Check if the build was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Building MeTA failed. Exiting"$RESET
  exit
fi

# # # # # # # # # # #
# Run the unit tests
# # # # # # # # # # #

echo -e $RESET $GREEN $NL"Running MeTA unit tests ..." $RESET
./unit-test --reporter=spec

# Check if the unit tests were successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"MeTA unit tests failed. Exiting"$RESET
  exit
fi

# # # # # # # # # # # # # # # # # #
# Download the language model files
# # # # # # # # # # # # # # # # # #

echo -e $RESET $GREEN $NL"Downloading the language model files ..." $RESET
META_MODELS_DIR=$TEMOTO_DIR/language_processors/meta/models
mkdir -p $META_MODELS_DIR

echo -e $RESET $GREEN"* Downloading the parser model to "$META_MODELS_DIR $RESET
wget -qO- https://github.com/meta-toolkit/meta/releases/download/v3.0.2/greedy-constituency-parser.tar.gz | tar zxv -C $META_MODELS_DIR

# Check if the download was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Failed to install the parser model file. Exiting"$RESET
  exit
fi

echo -e $RESET $GREEN"* Downloading the tagger model to "$META_MODELS_DIR $RESET
wget -qO- https://github.com/meta-toolkit/meta/releases/download/v3.0.2/greedy-perceptron-tagger.tar.gz | tar zxv -C $META_MODELS_DIR

# Check if the download was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Failed to install the tagger model file. Exiting"$RESET
  exit
fi

echo -e $NL"MeTA was installed successfully, run" $BOLD"catkin_make."

cd $PREV_DIR
