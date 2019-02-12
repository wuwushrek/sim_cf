# Gazebo for Crazy RealTime Protocol (CRTP) SITL and HITL

This is a flight simulator ROS package for the Crazyflie (by bitcraze) multirotors.

## How to set up the simulation environment.
In this section we will describe how to launch the gazebo environment and the crazyflie instance (linux compilation of crazyflie firmware) in both SITL and HITL. In the real environment, this will be equivalent to launching ```crazyflie_add_single.launch``` (launching a server and add a crazyflie to the server). So after this steps all the topics and the crazyflie will be on the ground in the simulation environment.

### Start the simulation in SITL mode
First, always starts with the script that will start gazebo. In this case it will be all launch files such as ```crazyflie_sim.launch```, ```multiple_cf_sim.launch```, ```max_cf_sitl.launch``` or any custom launch files created.

#### First start gazebo with a launch file similar as above

The launch files are simple to understand : [Gazebo](https://github.com/wuwushrek/sim_cf/blob/850133dbc854de9b04328bde99e39ac1435c00e0/crazyflie_gazebo/launch/max_cf_sitl.launch#L82-L95) is first launched with the appropriate world_name then you spawn [as much crazylfie](https://github.com/wuwushrek/sim_cf/blob/850133dbc854de9b04328bde99e39ac1435c00e0/crazyflie_gazebo/launch/max_cf_sitl.launch#L98-L187) object as you can. Make sure that the prefix are indexed correctly. WE need that order when we will launch the handler plugin.

Finally partitionned the crazyflie to different [handlers](https://github.com/wuwushrek/sim_cf/blob/850133dbc854de9b04328bde99e39ac1435c00e0/crazyflie_gazebo/launch/max_cf_sitl.launch#L190-L231) (if there is too much crazyflies). For example in the ```max_cf_sitl.launch``` we gave 4 crazyflies to the hanlder1 and those crazyflies are supposed to communicate via port 19950. the 3 others crazyflies are given to handler2 over port 19951. Then roslaunch them (for a single crazyflie):
```sh
roslaunch crazyflie_gazebo crazyflie_sim.launch
```
or if you want 7 crazyflies:
```sh
# use multiple_cf_sim.launch for 4 crazyflie with only 1handler
roslaunch crazyflie_gazebo max_cf_sitl.launch
```
#### Launch a crazyflie instance (the flight controller)
Some utilities script have been made to handle more easily swarm simulation. After the step above has been made and that you have compile the SITL as explained [here](https://github.com/wuwushrek/sim_cf#compilation-in-sitl-mode)
```sh
cd ~/catkin_ws/src/sim_cf/crazyflie_gazebo/scripts
# ./run_cfs.sh max_cfs port ip_address -> by default max_cfs=1 , port = 19950, ip_address=INADDR_ANY
# if crazyflie_sim.launch has been used
./run_cfs.sh
# if multiple_cf_sim.launch has been used
./run_cfs.sh 4
# if max_cf_sitl.launch has been used
./run_cfs 4 19950
./run_cfs 3 19951
```
[IMPORTANT] Then press PLAY button in gazebo and when ```gyrobias found``` is printed in the console where the crazylfie has been launched : you are done. you can start executing high level script.

#### Kill every crazyflie instances that has been created by run_cfs
This is useful before starting any simulation (launching gazebo)
```sh
cd ~/catkin_ws/src/sim_cf/crazyflie_gazebo/scripts
./terminate_cfs
```
### Start simulation in HITL mode
[IMPORTANT] the crazyflie_firmware must been compiled in HITL mode and must have been flash inside the crazylfie as explained [here](https://github.com/wuwushrek/sim_cf#compilation-in-hitl-mode). Once this is done, turn on the crazylfie and be sure that it can communicate with the computer over USB or Radio.
When using USB for HITL simulation with a single crazyflie connected :
```sh
roslaunch crazyflie_gazebo crazyflie_sim.launch uri:=usb://0
```
When using Radio with a single crazylfie connected :
```sh
roslaunch crazyflie_gazebo crazyflie_sim.launch uri:=radio://0/80/250K
```
Just note that in HITL with USB, a handler can only take in accound ONE crazylfie. So build your launch file using that information.

Then when the parameters and log have been found, press PLAY button in the gazebo engine to be able to start launching high level scripts. You should received a ```gyro bias found``` indicating that you can start playing around.

### Some high level tests
At this point, every script that can be executed in simulation is also supposed to work on the real crazyflie.
The two sections [here](https://github.com/wuwushrek/sim_cf#how-to-use-high-level-functionalities--example-in-test_high_levelpy) and [here](https://github.com/wuwushrek/sim_cf#how-to-send-setpoint-positionvelocityattitude) can also be applied as example.

The ghosts can also be added once gazebo has started if you want to visualize target_position topics.

### Using a joystick in simulation
When the gyrobias message has been received, you can use a joystick to control the crazylfie. This can be done by typing this in a console:
```sh
roslaunch crazylfie_gazebo joy_launch.launch cfPrefix:=cf1 positionTopic:=local_position 
```
Then FIRST press start button to 'arm motor' and then be technically able to use the joystick to control the crazyflie :
* Start : Arm the motors -> Always required after launching the node
* B     : Takeoff
* Y     : Landing
* X     : Change control mode to Position/Velocity control
          * Then press RB if you wish to start Position control
          * Press LB if you wish to start velocity control
* A     : change control mode to Attitude control
          * Press RB for starting Attitude control
* Left axes   : up-down
* Right axes  : left-right forward-backward
* LT, RT      : yaw control
* Cross keys  : increase sensitivity of current control

Basically to switch for example from a takeoff state to a position control without losing altitude, X then RB must be pressed in a shrt amount of time.

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
./run_cfs 4 19950
./run_cfs 3 19951
```
This script is a basic script launching all the crazyflie sitl instances.
```sh
cd ~/catkin_ws/src/sim_cf/crazyflie_gazebo/scripts/
# python max_uav_control.py [nbQuads] [tf_prefix]
python max_uav_control.py 7 cf
``` 

## How to improve the simulator performances

### Reduce the number of crazyflie
The obvious way is to reduce the number of crazyflie implied in gazebo simulator. Because of the collision avoidance algorithm, gazebo uses more and more cpu capacities when new objects are added (here when cf models are added).
Reducing the number of crazyflie can be simply done through the max_cf_sitl.launch or multiple_sim.launch examples.

### Play with the real_time_update_rate, max_step_size and iters
Usually the faster the simulation is running, the better it is. But your computer won't handle it and will reduce the real time factor. If you increase the update rate, just be sure that the gyro bias variance is not too big (which is not realistic). Increasing the iters tag should reduce it but will also affects the real time update.

### Fast control
...
