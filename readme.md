# About TeMoto

## What is it? ##
TeMoto is a telerobotics application development framework that is built upon ROS. TeMoto helps to integrate and manage different user-defined sequences of code (actions) and hardware resources like actuators, robots and sensors. 

## Why would I need it? ##
A teleoperation process may comprise many different subtasks. Ultimatlely the application developer solves the tasks by introducing one or a bunch of computer programs. The architecture of the composed software depends on the requirements of the problem at hand, ranging from simple to sophisticated in terms of:
* **Reusability** - Should/Can the developed software be reused for other applications?
* **Extendability** - How easy it is to add new features to the software?
* **Scalability** - Can the software be extended to multiple robots that work towards the same goal?
* **Flexibility** - How easy it is to interchange similar components in the system?

TeMoto is a framework that helps to create and manage applications that tend to be sophisticated in the above mentioned terms. So if you are developing a telerobotics application and the code starts to look like a bowl of sphagetti, then you may consider using TeMoto.

## How does it differ from traditional ROS applications? ##

Applications developed in ROS are traditionally managed solely by the user. An application in ROS can be a simple executable/node or a complex combination of different nodes that work together. Typically the latter is the case and in order to start the application, the user would need to either run each node manually (rosrun) or use launch files (roslaunch). After doing so, the node graph (configuration of running nodes) essentially remains static.

<img align="center" src="doc/images/ros.png">

<img align="center" src="doc/images/temoto.png">

![TeMoto](doc/images/temoto.png)

 Although ROS inherently promotes code reusability (the nodes can be reused in a different application), it provides no tools for managing the nodes during the run-time. 

The actions can be invoked during run-time.

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
