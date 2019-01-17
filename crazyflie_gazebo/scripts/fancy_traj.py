#!/usr/bin/env python

import sys
import rospy

from crazyflie_driver.msg import GenericLogData
from crazyflie_driver.msg import Position

import math

# Cube around the movement of the crazyflie
max_x = 1.0
max_y = 1.0
max_z = 1.6

# Position in Z for each planar movements
z_plan = 1.4

############################ Define some trajectories ####################################
# Define a circular trajectory. Each point returned should be send at the frequency freq
def circle_trajectory(freq, duration,r):
	x = list()
	y = list()
	t = list()
	nb_point = freq * duration
	for i in range(nb_point):
		t0 = 2 * math.pi * (1.0/freq)  * i
		x.append(r * math.cos(t0/duration))
		y.append(r * math.sin(t0/duration))
		t.append((1.0/freq)  * i)
	return (t,x,y)


# Define a fermat spiral trajectory. Each point returned should be send at the frequency freq
def fermat_spiral(freq , duration):
	x = list()
	y = list()
	t = list()
	nb_point = freq * duration
	for i in range(nb_point):
		t0 = i * (1.0/freq)
		x.append((max_x * t0 * t0 * math.cos(t0))/(duration * duration ))
		y.append((max_y * t0 * t0 * math.sin(t0))/ (duration * duration ))
		t.append(t0)
	return (t,x,y)


# Define a conical helix trajectory. Each point returned should be send at the frequency freq
def conical_helix(freq , duration ):
	x = list()
	y = list()
	z = list()
	t = list()
	nb_point = freq * duration
	for i in range(nb_point):
		t0 = i * (1.0/freq)
		x.append( (max_x * t0 * math.cos(20 * t0))/duration)
		y.append( (max_y * t0 * math.sin(20 * t0))/duration)
		z.append( 0.3 + ((t0/duration) * max_z ))
		t.append(t0)
	return (t,x,y,z)


# Define a uniform helix trajectory. Each point returned should be send at the frequency freq
def uniform_helix(freq , duration ):
	x = list()
	y = list()
	z = list()
	t = list()
	nb_point = freq * duration
	for i in range(nb_point):
		t0 = i * (1.0/freq)
		x.append( max_x * math.cos(1.0 * t0))
		y.append( max_y * math.sin(1.0 * t0))
		z.append( (t0/duration) * max_z )
		t.append(t0)
	return (t,x,y,z)
#########################################################################################

# local position callback [if need]
def local_position_callback(msg):
	global current_position
	current_position.x = msg.values[0]
	current_position.y = msg.values[1]
	current_position.z = msg.values[2]
	current_position.header = msg.header

if __name__ == '__main__':

	rospy.init_node('fancy_traj_node', anonymous=True)

	cf_prefix = "/cf1"

	# SUbscribe to get the local position of the crazyflie with prefix cf_prefix
	rospy.Subscriber(cf_prefix + "/local_position" , GenericLogData , local_position_callback)

	# The publisher to be able to control in position
	pubSetpointPos = rospy.Publisher(cf_prefix +"/cmd_position", Position , queue_size=10)

	# Frequency  between each point in the waypoint list
	freq_pub = 50
	rate = rospy.Rate(freq_pub)

	# INitialize global variable for actual position of crazyflie
	current_position = Position()

	# message used to store pos info to send
	next_pos = Position()

	# Choose a trajectory
	chosen_traj = raw_input('[ circle spiral conical_helix uniform_helix ]')

	# Takeoff first -> go to 0, 0 , 1
	while ((0-current_position.x)**2 + (0 - current_position.y)**2 + (1 - current_position.z)**2) > 1e-2 : # 10cm 
			next_pos.x = 0
			next_pos.y = 0
			next_pos.z = 1
			next_pos.yaw = 0.0
			next_pos.header.seq += 1
			next_pos.header.stamp = rospy.Time.now()
			pubSetpointPos.publish(next_pos)
			rate.sleep()

	t = list()
	x = list()
	y = list()
	z = list()
	if chosen_traj == 'circle':
		(t , x , y) = circle_trajectory(freq_pub , 15  , 1)
		z = [ z_plan for val in range(len(x))]
	elif chosen_traj == 'spiral':
		(t , x , y) = fermat_spiral(freq_pub , 15 )
		z = [ z_plan for val in range(len(x))]
	elif chosen_traj == 'conical_helix':
		(t , x , y , z ) = conical_helix(freq_pub,35)	
	else :
		(t , x , y , z ) = uniform_helix(freq_pub,35)
	for i in range(len(x)):
		print ("t,x,y,z ", t[i] , x[i], y[i], z[i])
		
	# first go to the first point of the trajectory
	while ((x[0]-current_position.x)**2 + (y[0] - current_position.y)**2 + (z[0] - current_position.z)**2) > 1e-2 : # 10cm 
		next_pos.x = x[0]
		next_pos.y = y[0]
		next_pos.z = z[0]
		next_pos.yaw = 0.0
		next_pos.header.seq += 1
		next_pos.header.stamp = rospy.Time.now()
		pubSetpointPos.publish(next_pos)
		rate.sleep()

	# Then start the trajectory
	for i in range(len(x)):
		next_pos.x = x[i]
		next_pos.y = y[i]
		next_pos.z = z[i]
		next_pos.yaw = 0.0
		next_pos.header.seq += 1
		next_pos.header.stamp = rospy.Time.now()
		pubSetpointPos.publish(next_pos)
		rate.sleep()

	# Sleep for some time
	for i in range(100):
		rate.sleep()

	# Land -> go to 0 0 0.1
	while ((0-current_position.x)**2 + (0 - current_position.y)**2 + (0.1 - current_position.z)**2) > 1e-2 : # 10cm 
			next_pos.x = 0
			next_pos.y = 0
			next_pos.z = 0.1
			next_pos.yaw = 0.0
			next_pos.header.seq += 1
			next_pos.header.stamp = rospy.Time.now()
			pubSetpointPos.publish(next_pos)
			rate.sleep()
