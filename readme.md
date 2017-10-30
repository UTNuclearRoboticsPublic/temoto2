# TeMoto 2 install guide

## Ubuntu 16.04
**The first thing is to set up the language processor:**
1. Clone "MeTA: ModErn Text Analysis" [repository](https://github.com/meta-toolkit/meta/)
1. Follow the MeTA [Ubuntu 15.10 Build Guide](https://github.com/meta-toolkit/meta/#ubuntu-1510-build-guide)
1. Download the model files: 
   * [Parser model file](https://github.com/meta-toolkit/meta/releases/download/v3.0.2/greedy-constituency-parser.tar.gz)
   * [perceptron tagger model file](https://github.com/meta-toolkit/meta/releases/download/v3.0.2/greedy-perceptron-tagger.tar.gz)
1. Move the downloaded files into *"temoto_2/include/TTP/language_processors/meta/models"* and extract them
