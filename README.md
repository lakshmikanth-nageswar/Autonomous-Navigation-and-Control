# Autonomous Navigation and Control

This repo consists of my implementation of an autonomous bot using ROS framework. The developed algorithms are tested in Gazebo simulator.

## src is the latest update:
###  to run it do the following:
####    ->clone the repo
####    ->create an empty workspace
####    ->add an empty src to the ws
####    ->catkin build this ws
####    ->replace the src with the cloned src
####    ->chmod +x the python scripts
####    ->change the path in env1.world by finding and replacing lakshmikanth/... as required
####    
####    ->$ source devel/setup.bash
####    ->$ roslaunch final_description controls.launch
####    -> Press the play button on the bottom
####    ->$ rosrun final_description arrow.py/ball_detection.py/image.py in that order
