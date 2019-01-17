#!/bin/bash

if [ -z "$1" ]
then
	max_cfs=1
else
	max_cfs=$1
fi

if [ -z "$2" ]
then
	udp_port=19950
else
	udp_port=$2
fi

if [ -z "$3" ]
then
	ip_address=INADDR_ANY
else
	ip_address=$3
fi

counter=1
while [ $counter -le $max_cfs ]
do
	./../../crazyflie-firmware/sitl_make/build/cf2 $counter $udp_port $ip_address &
	sleep 0.3
	((counter++))
done