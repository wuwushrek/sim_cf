# Gazebo Simulation for Crazyflie CRTP 

This is a flight simulator for the Crazyflie (by bitcraze) multirotors. It principally uses the motor model and some sensors model from RotorS https://github.com/ethz-asl/rotors_simulator . In addition, we added support for both SITL and HITL simulation of the crazyflie using some information gathered from Allan Standard Deviation analysis of each cf2 sensors.

## Dependencies

This manual has been tested on a clean Ubuntu 16.04 LTS installation.

### Common dependencies

Install basics dependencies and the protobuf library, which is used as interface to Gazebo.
```sh
# Basic dependencies
sudo apt-get install git zip qtcreator cmake build-essential genromfs ninja-build -y
# Protobuf , eigen3 and google-glog dependencies
sudo apt-get install protobuf-compiler libgoogle-glog-dev libeigen3-dev libxml2-utils
```

### ROS dependencies

Install ROS for [Ubuntu 16.04](http://wiki.ros.org/kinetic/Installation/Ubuntu). This tutorial should also install gazebo7 and gazebo7 dev if the desktop full version has been selected.  

You will also need mav-comm and joy package installed on your computer in order to compile the crazyflie_gazebo ROS package
```sh
# For Ros Kinetic (Ubuntu 16.04)
sudo apt-get install ros-kinetic-mav-comm ros-kinetic-joy
```

Also install the package rqt_multiplot necessary for plotting useful data:
```sh
sudo apt-get install ros-kinetic-rqt-multiplot
```

Create and initialize ROS workspace if needed
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### Crazyflie client python dependencies
In order to be able to flash the crazyflie with the HITL modifications or the default crazyflie code, you will need to have cfclient installed on your computer. If cfclient is not already installed on your computer, follow this (pip3 is needed) :
```sh
cd ~/
git clone https://github.com/bitcraze/crazyflie-clients-python.git
cd crazyflie-clients-python
pip3 install --user -e .
```

## Build crazyflie_gazebo package

Assuming your catkin workspace is in ~/catkin_ws, We need the messages in crazyflie_driver of crazyflie_ros. So clone all the repo if you also want to use ros-related functionnality by crazyflie_ros with the simulation. Therefore first clone the latest crazyflie_ros package from whoenig: 
```sh
cd ~/catkin_ws/src
git clone https://github.com/whoenig/crazyflie_ros.git
cd crazyflie_ros
git submodule update --init --recursive
```
Then execute the following commands in order download and build crazyflie_gazebo package :
```sh
git clone https://github.com/wuwushrek/sim_cf.git
cd sim_cf/
git submodule update --init --recursive
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Crazyflie-firmware package [Required to do SITL compilation below]

This file contains the source code from bitcraze of Crazyflie. Macros + new files for simulating sensors and others have been added in order to make HITL/SITL with minimum modification (no modification on the overall behavior) of the original code. A new task has been added that replaces syslink/usblink (depending of the type of simulation) and implement a socket client that communicates via UDP with the gazebo server.

### Compilation in SITL mode

For SITL, a high performance computer is needed since gazebo simulation and the crazyflie source software will be launched on the same computer. The possibility of using another computer for crazyflie simulation is also implemented. The computer performing the crazyflie simulation and the computer performing gazebo rendering must be able to communicate via socket. No need to install ROS on the first one.
Follow this to build crazyflie in sitl mode : 
```sh
cd ~/catkin_ws/src/sim_cf/crazyflie-firmware/sitl_make
mkdir build
cd build
cmake ..
make
``` 

### Compilation in HITL mode

To be able to test HITL mode, you may need to compile the source code in HITL mode. Execute the following commands : 
```sh
cd ~/catkin_ws/src/sim_cf/crazyflie-firmware
make PLATFORM=hitl
# Start crazyflie in bootloader mode then 
make cload
```
If you wished to compile it in default mode for tests in the real world:
```sh
cd ~/catkin_ws/src/sim_cf/crazyflie-firmware
make PLATFORM=cf2
# Start crazyflie in bootloader mode then 
make cload
```

## Crazyflie_gazebo package

Follow documentation on https://github.com/wuwushrek/sim_cf/tree/master/crazyflie_gazebo in order to communicate with the SITL/HITL instance and start playing with the simulation environment.
![alt text](https://github.com/wuwushrek/sim_cf/blob/multi-uav-final/7cfs.gif)

# [Salle LIX] Flying crazyflies out of simulation (Using LPS)

```sh
cd ~/catkin_ws/src/sim_cf
git pull
```

Before everything else, compile and flash the crazyflies with PLATFORM=cf2 as shown above. You can use either the crazyflie-firmware of this package or the original one from bitcraze. Then turn on the crazyflies and put them in the arena.

Now open a console, launch and establish connection with the crazyflie using the launch files ```crazyflie_add_*``` from crazyflie_gazebo package : 
```sh
# For connection with only a single crazyflie -> Modify the uri depending on your platform
roslaunch crazyflie_gazebo crazyflie_add_single.launch
```
or for launching 2 crazyflies (the script can be easily modified to launch as much crazyflies as desired, just look at the difference between ```crazyflie_add_single.launch``` and ```crazyflie_add_multi.launch```):
```sh
# For connection with 2 crazyflies -> Modify the uri depending on your platform
roslaunch crazyflie_gazebo crazyflie_add_multi.launch
```

Now if you want to get more information from the crazyflie, setting to True [one of these parameters](https://github.com/wuwushrek/sim_cf/blob/7d4a882097dff122cd7ff90e734a0319e929fdeb/crazyflie_gazebo/launch/crazyflie_add_single.launch#L8-L17) will do the trick. or you can also add your custom logs as done [here](https://github.com/wuwushrek/sim_cf/blob/7d4a882097dff122cd7ff90e734a0319e929fdeb/crazyflie_gazebo/launch/crazyflie_add_single.launch#L40-L43).

Also note that [the prefix](https://github.com/wuwushrek/sim_cf/blob/7d4a882097dff122cd7ff90e734a0319e929fdeb/crazyflie_gazebo/launch/crazyflie_add_single.launch#L5) argument here is the prefix that will be applied to all of your ROS topics and service created by crazyflieROS. So in order to be able to launch whatever high level function you want, check that the topic you want to extract or send information are prefixed according to that prefix. For example this is used [here](https://github.com/wuwushrek/sim_cf/blob/7d4a882097dff122cd7ff90e734a0319e929fdeb/crazyflie_gazebo/scripts/test_high_level.py#L23) in the ```test_high_level script.py``` to say that all the topics / custom logs will start with ```/cf1```.


# How to use high level functionalities : Example in test_high_level.py
High level functionalities includes takeoff, landing, goTo, uploadTrajectory, startTrajectory and etc... A set of example about using them can be found in the [crazyflie_ros package](https://github.com/whoenig/crazyflie_ros). To be able to use this example script, the connection with the crazyflie has to already be established (either in simulation or on a real crazyflie). the ```tf_prefix``` should be the same as the second argument of the constructor [here](https://github.com/wuwushrek/sim_cf/blob/7d4a882097dff122cd7ff90e734a0319e929fdeb/crazyflie_gazebo/scripts/test_high_level.py#L23).

```sh
cd ~/catkin_ws/src/sim_cf/crazyflie_gazebo/scripts/
python test_high_level.py
```

This script file is a simple python file that executes a set of example of high level functionnality. These high level functionality are made possible via ROS client/server and ROS topics.
First initialize a crazyflie instance (a simple class giving high level ROS functionnality to quickly connect to these topics) like it is done on [this line](https://github.com/wuwushrek/sim_cf/blob/7d4a882097dff122cd7ff90e734a0319e929fdeb/crazyflie_gazebo/scripts/test_high_level.py#L23). Then enable the high level functionnality using [commander/enHighLvl params](https://github.com/wuwushrek/sim_cf/blob/7d4a882097dff122cd7ff90e734a0319e929fdeb/crazyflie_gazebo/scripts/test_high_level.py#L25). Finally have a look at the prototype of functions [here](https://github.com/wuwushrek/sim_cf/blob/master/crazyflie_gazebo/scripts/crazyflie.py) in order to know the different high level functions availabled. An instance of high level functionality is the taking off functionality. Taking off only required [these lines](https://github.com/wuwushrek/sim_cf/blob/7d4a882097dff122cd7ff90e734a0319e929fdeb/crazyflie_gazebo/scripts/test_high_level.py#L27) if and only if all the previous steps have been done.

# How to send setpoint position/velocity/attitude
To send a target in position, positions information have to be send continuously on the cf_prefix/cmd_position topic. for velocity on cf_prefix/cmd_vel. An example of how to do it can be found in the [fancy_traj.py](https://github.com/wuwushrek/sim_cf/blob/master/crazyflie_gazebo/scripts/fancy_traj.py). This file generates some basic trajectory and send the different points of the trajectory over cmd_position topic. To use it :
```sh
cd ~/catkin_ws/src/sim_cf/crazyflie_gazebo/scripts
python fancy_traj.py
# write the trajectory you want amongst the proposed list and type enter
```

# How to monitor real crazyflie using gazebo environment
In this section, it is possible to monitor in the gazebo environment what the real crazyflie is doing. Basically just showing in the virtual environment the actual position/orientation of the crazyflie and a ghost following it target position and orientation. To use that, ``` be sure that you are already connected to crazyflie and that you have a custom log that gives you at least position and/or euler angles```, in that case :
```sh
roslaunch gazebo_ros empty_world.launch world_name:=$HOME/catkin_ws/src/sim_cf/crazyflie_gazebo/worlds/salle_lix.world

# Spawn a ghost that will follow target point sent to the crazyflie
roslaunch crazyflie_gazebo spawn_ghost.launch allow_collision:=false modelName:=noCollisionGHost poseTopic:=/cf1/target_position

# Spawn a ghost that will follow the true position/orientation of the crazyflie in the real world
roslaunch crazyflie_gazebo spawn_ghost.launch allow_collision:=true modelName:=realcf1 poseTopic:=/cf1/local_position color_prop_front:=Blue color_prop_back:=Red
```
Here, the poseTopic should target the topic you want the ghosts to follow. So the prefix used should be identical to the prefix  where the position/orientation of the crazyflie is published. You can add as much ghosts as wished for different crazyflies in the real environment.

# Making some plots rqt_multiplot
When the crazyflie has been put in the arena, it is possible to visualize its 3D position in real time using rqt_multiplot. Follow the description on [this ROS page](http://wiki.ros.org/rqt_multiplot) if you wish to customize your own plots. A configuration file that plots x(t),y(t),z(t),y(x),z(y),z(x),roll(t),pitch(t),yaw(t) and their target is already been proposed for a crazyflie prefixed by cf1. To change the prefix, replace all occurence of ```cf1``` in the [configuration file](https://github.com/wuwushrek/sim_cf/blob/master/crazyflie_gazebo/launch/mylog_plot.xml) by the prefix wished. Then launch the following command :
```sh
rqt_multiplot --multiplot-config ~/catkin_ws/src/sim_cf/crazyflie_gazebo/launch/mylog_plot.xml
```
![alt text](https://github.com/wuwushrek/sim_cf/blob/master/log_plot.png)
