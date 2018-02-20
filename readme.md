# TeMoto 2 install guide

## Ubuntu 16.04
**1. Clone TeMoto to your catkin_workspace/src directory**
```
cd <your_catkin_ws>/src
git clone https://github.com/UTNuclearRobotics/temoto2.git
```

**2. Install TeMoto dependancies via installer script**
```
bash ./temoto2/temoto_2/scripts/install_temoto_deps.sh
```

**3. Install MeTA natural language processor via installer script**

This script will 
* install MeTA related dependancies (asks your password)
* clone, build and test MeTA
* download the language model files

PS: The whole process may take up to few minutes.
```
bash ./temoto2/temoto_2/scripts/install_meta.sh
```

**4. Build your catkin workspace**
```
cd <your_catkin_ws>
catkin_make
```
