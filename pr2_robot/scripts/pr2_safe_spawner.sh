#! /bin/bash
# This script safely launches ros nodes with buffer time to allow param server population
xterm -e roslaunch pr2_robot pick_place_demo.launch & sleep 10 &&
sleep 5
xterm -e roslaunch pr2_moveit pr2_moveit.launch & sleep 20 &&
sleep 5
xterm -e rosrun pr2_robot pr2_motion
