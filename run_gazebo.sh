#!/bin/bash

sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall roscore
sudo killall rosmaster

roslaunch ackermann_vehicle_gazebo ackermann_vehicle_teleop.launch 

