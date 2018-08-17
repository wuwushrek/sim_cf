# Gazebo for Crazy RealTime Protocol (CRTP) SITL and HITL

This is a flight simulator ROS package for the Crazyflie (by bitcraze) multirotors.

## Basic Usage

Gazebo node and ROS must be launched BEFORE any other crazyflie SITL instance.
However in HITL, THe crazyflie must be plugged on the computer before starting gazebo.

A simple launch file that allows you to do position, attitude control  with high level command such as
taking off and landing can be tested with the following command :

### For an example with a single quad in SITL
First launch the following gazebo launch file in a console : 
```sh
roslaunch crazyflie_gazebo crazyflie_sitl.launch
```
Then Open a new console and launch an instance of cf2 :
```sh
#./cf2 id port ip_address --> currently 1 - 19950 - INADDR_ANY are the default id/port/address
./cf2
# EQUIVALENT TO :
./cf2 1 19950 INADDR_ANY
```
The id argument here is used as a suffix of the tf_prefix of every crazyflie in the simulation environment. After this steps, move on ```Some scripts tests``` section to try some high level functionality.

### For an example with multiples crazyflie
The following experience was realized on a Ubuntu 16.04 computer, Intel® Core™ i7-8650U CPU @ 1.90GHz × 8 and with 15.6 GiB of RAM.
The performances of multiple crazyflie scenario depend heavily on the computer capacity (Mainly the cpu).You may need to reduce the number of crazyflie spawned here in order to have good enough performances for the control. 

Here is an example implying 7 crazyflie SITL instances:
First open a console and execute the following commands :
```sh
roslaunch crazyflie_gazebo max_cf_sitl.launch
```
This will launch gazebo, the 7 crazyflie models spawned inside ``max_cf_sitl.launch`` + two handlers. the first one communicates with cf1 to cf4 on port 19950 and the second one with cf5 to cf7 on port 19951. The tf_prefix used here is ``cf``.

Finally open a new console, go to the crazyflie_gazebo/scripts/ directory. Then execute the following command :
```sh
cd ~/catkin_ws/src/sim_cf/crazyflie_gazebo/scripts/
./cfs_instances.sh
```
This script is a basic script launching all the crazyflie sitl instances. YOU WILL NEED TO PRESS ENTER after each cf instance has been initialized. For the moment, this is just a workaround in order to pause between every cf initialization.


## Some scripts tests in SITL

### For a single crazyflie
```sh
cd ~/catkin_ws/src/sim_cf/crazyflie_gazebo/scripts/
python test_high_level.py
```
To visualize the crazyflie current pose and target pose in an RVIZ environment, just open a new console and then enter the following commands : 
```sh
cd ~/catkin_ws/src/sim_cf/crazyflie_gazebo/launch
rosrun rviz rviz -d crazyflie_visu.rviz
```

### In case of multiple crazyflies
If you have followed the multiple crazyflie example above, That means you have spawned 7 crazyflies with the prefix cf, just launch the following commands :
```sh
cd ~/catkin_ws/src/sim_cf/crazyflie_gazebo/scripts/
# python max_uav_control.py [nbQuads] [tf_prefix]
python max_uav_control.py 7 cf
``` 

## How to improve the simulator performances

### Reduce the number of crazyflie
The obvious way is to reduce the number of crazyflie implied in gazebo simulator. Because of the collision avoidance algorithm, gazebo uses more and more cpu capacities when new objects are added (here when cf models are added).
Reducing the number of crazyflie can be simply done through the max_cf_sitl.launch or multiple_cf.launch examples. You can get inspired from ```cfs_instances.sh``` and the ```max_uav_control.py``` script to chose your own number of quads.

### Play with the real_time_update_rate, max_step_size and iters
Usually the faster the simulation is running, the better it is. But your computer won't handle it and will reduce the real time factor. If you increase the update rate, just be sure that the gyro bias variance is not too big (which is not realistic). Increasing the iters tag should reduce it but will also affects the real time update.

### Fast control
...