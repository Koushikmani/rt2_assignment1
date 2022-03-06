# RESEARCH TRACK2 - Assignment 1:ros2

## Koushikmani Maskalmatti Lakshman (S5053566)

The reason for this package is to gives with nodes written in ROS1 bundle to control a non-holonomic mobile robot in a Gazebo environment. The client can decide to begin the robot and to stop it. While the client chooses to make moving the robot, an arbitrary posture is choosen and the robot begin moving that way. This development go on until the robot reaches the objective goal and another irregular posture is sent.

## To compile and run the package

To communicate with ROS1, we really want three distinct steps

Launch the part from ROS1, so in the shell where you obtained ROS1

         roslaunch rt2_ass1_std sim_ros2.launch
         
In a second shell source ros1 and ros2 and run the bridge:

         ros2 run ros1_bridge dynamic_bridge
         
In a third shell source ros2 and run:

         roslaunch rt2_assignment1 sim_launch.py


## Description of the branch. 
In this organizer you can find:
launch: having the launch file sim_launch.py,expected to launch a compartment and to stack the parts into.
src: contains two nodes implemented as c++ file 
position_server:  carries out the server for an arbitrary (x,y,theta) present.
state_machine: makes the demand of a new goal and it send the request as a goal to go_to_point action server. 

