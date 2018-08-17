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
![alt text](https://github.com/wuwushrek/sim_cf/blob/multi-uav-final/7cfs.gif)

# [Salle LIX] Flying with a single real crazyflie (Using LPS)

```sh
cd ~/catkin_ws/src/sim_cf
git pull
```

Before everything else, Turn on the crazyflie and put it in the arena.

Now open a console and launch the crazyflie server using the following command : 
```sh
roslaunch crazyflie_gazebo crazyflie_server
```
This command allows you to open a 'server' that will be waiting for removal or addition of different crazyflies.

Open  a new console and launch the following command in order to add a crazyflie (therefore to be able to communicate with the crazyflie using ROS topics) : 
```sh
roslaunch crazyflie_gazebo crazyflie_add
```
Now if you want to get more information from the crazyflie, setting to True [one of these parameters](https://github.com/wuwushrek/sim_cf/blob/5b069ef0c6aa41eb450cd85d9821d2695f126701/crazyflie_gazebo/launch/crazyflie_add.launch#L8-L19) will do the trick. Also note that [the prefix](https://github.com/wuwushrek/sim_cf/blob/5b069ef0c6aa41eb450cd85d9821d2695f126701/crazyflie_gazebo/launch/crazyflie_add.launch#L7) argument here is the prefix that will be applied to all of your ROS topic. That means that for example to listen to the pose of the crazyflie, you should do :
```sh
# tf_prefix in the current crazyflie_add.launch is equal to /cf1
rostopic echo /tf_prefix/pose
```

After the crazyflie has been successfully added, you can now launch whatever high level function you want. For example, check the script [test_high_level.py](https://github.com/wuwushrek/sim_cf/blob/multi-uav-final/crazyflie_gazebo/scripts/test_high_level.py). Just be sure that [here](https://github.com/wuwushrek/sim_cf/blob/5b069ef0c6aa41eb450cd85d9821d2695f126701/crazyflie_gazebo/scripts/test_high_level.py#L23) you have set correctly the tf_prefix that match the value used in crazyflie_add.launch

# How does the test_high_level.py works

```sh
cd ~/catkin_ws/src/sim_cf/scripts/
python test_high_level.py
```

This script file is a simple python file that execute a set of example of high level functionnality. These high level functionnality are made possible via ROS client/server and ROS topics.
First initialize a crazyflie instance (a simple class giving high level ROS functionnality to quickly connect to these topics) like it is done on [this line](https://github.com/wuwushrek/sim_cf/blob/a033a78135ccd46f40b161d1d30a4faa2d7ee5a7/crazyflie_gazebo/scripts/test_high_level.py#L23). Then enable the high level functionnality using [commander/enHighLvl params](https://github.com/wuwushrek/sim_cf/blob/a033a78135ccd46f40b161d1d30a4faa2d7ee5a7/crazyflie_gazebo/scripts/test_high_level.py#L25). Finally have a look at the prototype of functions [here](https://github.com/wuwushrek/sim_cf/blob/multi-uav-final/crazyflie_gazebo/scripts/crazyflie.py) in order to know the different high level function availabled. An instance of high level functionality is the taking off functionality. Taking off only required [these lines](https://github.com/wuwushrek/sim_cf/blob/4b3e47e279c55665cca214cda938f3a818c394cd/crazyflie_gazebo/scripts/test_high_level.py#L27-L28) if and only if all the previous steps have been done. I guess you know what to do for other high level functionality ;) Check again the python file for more details/

# How to monitor real crazyflie using gazebo environment
[IMPORTANT]
First thing to do is to check the [crazyflie_ghost1](https://github.com/wuwushrek/sim_cf/blob/multi-uav-final/crazyflie_gazebo/models/crazyflie_ghost1/crazyflie_ghost1.sdf) and the [crazyflie_ghost2](https://github.com/wuwushrek/sim_cf/blob/multi-uav-final/crazyflie_gazebo/models/crazyflie_ghost2/crazyflie_ghost2.sdf). You will have to check if every <uri> that are pointing to meshes are pointing to the correct path. Simply try to find all occurence of ```<uri>/home/**user**/*``` and replace every occurence of that user name by your user name. An example of line to modify can be seen [here](https://github.com/wuwushrek/sim_cf/blob/9ced20450890b507c24acbf39986d8cb9e87cabe/crazyflie_gazebo/models/crazyflie_ghost2/crazyflie_ghost2.sdf#L35). Change ```franck``` with whatever your user name is.

For an experiment with on crazyflie, you may also want to check that the prefix used [here](https://github.com/wuwushrek/sim_cf/blob/9ced20450890b507c24acbf39986d8cb9e87cabe/crazyflie_gazebo/models/crazyflie_ghost2/crazyflie_ghost2.sdf#L319) and [here](https://github.com/wuwushrek/sim_cf/blob/9ced20450890b507c24acbf39986d8cb9e87cabe/crazyflie_gazebo/models/crazyflie_ghost1/crazyflie_ghost1.sdf#L321) matches the prefix you choose in the launch file.

When the crazyflie has been put in the arena, it is possible to visualize its 3D position using GAZEBO or RVIZ. Using gazebo the only step to do is launching the following commands :
```sh
roslaunch crazyflie_gazebo crazyflie_real_gazebo.launch
```
Then click on the play button on the gazebo windows in order to have the virtual crazyflie following the real one. At the beginning you should see 2 crazyflie : The first one is kind of transparent and will represents the desired state for the real crazyflie and the second one will represent the current estimate state for the real crazyflie.