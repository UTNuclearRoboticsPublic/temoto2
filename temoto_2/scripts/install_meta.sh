#!/bin/bash

RED="\e[31m"
YELLOW="\e[33m"
GREEN="\e[32m"
BOLD="\e[1m"
NL="\n"
RESET="\e[39m\e[0m"

TEMOTO_DIR=$(rospack find temoto_2)

# Check if the package was found or not
if [[ $TEMOTO_DIR = "" ]]; then
  echo -e $RED"Could not find the temoto_2 package. Have you built it?"$RESET
  exit
fi

META_SRC_DIR=TEMOTO_DIR/language_processors/meta

# Update and install the dependancies
echo -e $YELLOW $NL"Insert your sudo password to update and install the dependencies (g++, git, cmake, make, libjemalloc-dev, zlib1g-dev, wget)"$RESET
sudo apt update
sudo apt install g++ git cmake make libjemalloc-dev zlib1g-dev wget


# # # # # # # # # 
# Install MeTA
# # # # # # # # #

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

# Run the unit tests
echo -e $RESET $GREEN $NL"Running MeTA unit tests ..." $RESET
./unit-test --reporter=spec

# Check if the unit tests were successful or not
if [[ $? = 1 ]]; then
  echo -e $RED $BOLD"MeTA unit tests failed. Exiting"$RESET
  exit
fi

# Download the language model files
echo -e $RESET $GREEN $NL"Downloading the language model files ..." $RESET
MODELS_DIR=$TEMOTO_DIR/include/TTP/language_processors/meta/models
wget -qO- https://github.com/meta-toolkit/meta/releases/download/v3.0.2/greedy-constituency-parser.tar.gz | tar zxv -C $MODELS_DIR
wget -qO- https://github.com/meta-toolkit/meta/releases/download/v3.0.2/greedy-perceptron-tagger.tar.gz | tar zxv -C $MODELS_DIR

cd $PREV_DIR
