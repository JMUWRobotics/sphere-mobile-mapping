#!/bin/sh
catkin_make clean && rm -r build/; 
catkin_make --pkg state_estimator_msgs && catkin_make
