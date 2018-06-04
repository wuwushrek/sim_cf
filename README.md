# Gazebo Simulation for Crazyflie CRTP 

This is a flight simulator for the Crazyflie (by bitcraze) multirotors. It principally uses the motor model and some sensors model from RotorS https://github.com/ethz-asl/rotors_simulator . In addition, we added support for both SITL and HITL simulation of the crazyflie

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

## Build crazyflie_gazebo package

Assuming your catkin workspace is in ~/catkin_ws, do the following commands in order to build crazyflie_gazebo package : 
```sh
cd ~/catkin_ws/src
git clone https://github.com/wuwushrek/sim_cf.git
cd sim_cf/
git submodule update --init --recursive
# [IMPORTANT] This is needed in order to stop catkin from building crazyflie-clients-python
touch crazyflie-clients-python/CATKIN_IGNORE
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Crazyflie-firmware package

This file contains the source code from bitcraze of Crazyflie. Macros + new files for simulating sensors and others have been added in order to make HITL/SITL with minimum modification (no modification on the overall behavior) of the original code.

### Compilation in HITL mode

To be able to test HITL mode, you may need to compile the source code in HITL mode. This can be done by a small modification of [these lines](https://github.com/wuwushrek/crazyflie-firmware/blob/4789162e399d8f19b21a078f5158a603cf83e15c/Makefile#L32-L33) : changes ```stock``` and ```cf2``` by ```hitl```. then execute the following commands : 
```sh
cd ~/catkin_ws/src/sim_cf/crazyflie-firmware
make
# Start crazyflie in bootloader mode then
make cload
```

### Compilation in SITL mode

For SITL, a high performance computer is needed since gazebo simulation and the crazyflie source software will be launched on the same computer. The possibility of using another computer for crazyflie simulation is also implemented. The computer performing the crazyflie simulation and the computer performing gazebo simulation must be able to communicate via socket. No need to install ROS on the first one.
Follow this to build crazyflie in sitl mode : 
```sh
cd ~/catkin_ws/src/sim_cf/crazyflie-firmware/sitl_make
mkdir build
cd build
cmake ..
make
``` 

Then launch cf2 with the following command : 
```sh
#./cf2 port ip_address --> currently INADDR_ANY - 19950 are the default address/port
./cf2
```

## Crazyflie_gazebo package

Follow documentation on https://github.com/wuwushrek/sim_cf/tree/master/crazyflie_gazebo in order to communicate with the SITL/HITL instance and start playing with the simulation environment.