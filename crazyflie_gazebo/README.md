# Gazebo for Crazy RealTime Protocol (CRTP) SITL and HITL

This is a flight simulator ROS package for the Crazyflie (by bitcraze) multirotors.

## Basic Usage

The crazyflie HITL/SITL instance must be run before doing anything else related to gazebo and ROS.

A simple launch file that allows you to do position, attitude control  with high level command such as
taking off and landing can be tested with the following command :

```sh
roslaunch crazyflie_gazebo crazyflie_with_joy.launch
```

## Basic configuration



## Some scripts tests

```sh
cd ~/catkin_ws/src/Modal2018/crazyflie_gazebo/scripts/
python test_high_level.py
```