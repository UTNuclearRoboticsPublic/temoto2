# TeMoto 2 install guide

## Ubuntu 16.04
**The first thing is to set up the language processor:**
1. Clone "MeTA: ModErn Text Analysis" [repository](https://github.com/meta-toolkit/meta/)
1. Follow the MeTA [Ubuntu 15.10 Build Guide](https://github.com/meta-toolkit/meta/#ubuntu-1510-build-guide)
1. Download the model files: 
   * [Parser model file](https://github.com/meta-toolkit/meta/releases/download/v3.0.2/greedy-constituency-parser.tar.gz)
   * [perceptron tagger model file](https://github.com/meta-toolkit/meta/releases/download/v3.0.2/greedy-perceptron-tagger.tar.gz)
1. Move the downloaded files into *"temoto_2/include/TTP/language_processors/meta/models"* and extract them

**Copy-paste instructions:**

```
META_SRC_DIR=~/meta
sudo apt update
sudo apt install g++ git cmake make libjemalloc-dev zlib1g-dev wget
```

```
mkdir -p $META_SRC_DIR
cd $META_SRC_DIR
git clone https://github.com/meta-toolkit/meta.git ./
git submodule update --init --recursive
mkdir $META_SRC_DIR/build
cp $META_SRC_DIR/config.toml $META_SRC_DIR/build/config.toml
cd $META_SRC_DIR/build
cmake ../ -DCMAKE_BUILD_TYPE=Release
make -j8
./unit-test --reporter=spec
MODELS_DIR=`rospack find temoto_2`/include/TTP/language_processors/meta/models
wget -qO- https://github.com/meta-toolkit/meta/releases/download/v3.0.2/greedy-constituency-parser.tar.gz | tar zxv -C $MODELS_DIR
wget -qO- https://github.com/meta-toolkit/meta/releases/download/v3.0.2/greedy-perceptron-tagger.tar.gz | tar zxv -C $MODELS_DIR

```
