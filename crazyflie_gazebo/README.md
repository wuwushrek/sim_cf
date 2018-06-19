# Gazebo for Crazy RealTime Protocol (CRTP) SITL and HITL

This is a flight simulator ROS package for the Crazyflie (by bitcraze) multirotors.

## Basic Usage

The crazyflie HITL/SITL instance must be run before doing anything else related to gazebo and ROS.

A simple launch file that allows you to do position, attitude control  with high level command such as
taking off and landing can be tested with the following command :

### For an example with a single quad
Open a new console and launch an instance of cf2 :
```sh
#./cf2 port ip_address --> currently INADDR_ANY - 19950 are the default address/port
./cf2
```
Then launch the following launch file in another console : 
```sh
roslaunch crazyflie_gazebo crazyflie_with_joy.launch
```

### For an example with two crazyflie
Open a two new consoles and launch two instance of cf2 with differents address or port : 
```sh
#./cf2 port ip_address --> currently INADDR_ANY - 19950 are the default address/port
./cf2
```
The second console
```sh
#./cf2 port ip_address --> currently INADDR_ANY - 19950 are the default address/port
./cf2 19951 INADDR_ANY
```
Then refer to the multiple_cf.launch for spawning multiple quads in gazebo. You can launch it using the following commands : 
```sh
roslaunch crazyflie_gazebo multiple_cf.launch
```

## Basic configuration



## Some scripts tests

For a single quad test, run the following script : 
```sh
cd ~/catkin_ws/src/sim_cf/crazyflie_gazebo/scripts/
python test_high_level.py
```

For a simulation with two quadcopter, run the following script : 
```sh
cd ~/catkin_ws/src/sim_cf/crazyflie_gazebo/scripts/
python test_high_level_multi_cf.py
```