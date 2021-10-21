#!/bin/bash

# Load ROS parameters
roslaunch gym_gazebo train.launch

# Run train script
rosrun gym_gazebo train.py
