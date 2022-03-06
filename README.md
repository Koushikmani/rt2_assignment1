# RESEARCH TRACK2 - Assignment 1: action server

 Koushikmani Maskalmatti Lakshman(S5053566)
The aim of this package is to control a non-holonomic mobile robot in a Gazebo environment. The user can choose to make the robot start moving or stop it. When the user gives the command to start moving, a random pose is chosen and the robot start moving. The user has the possibility to stop the robot, even if a random pose has been already chosen. When the user gives the command to stop the robot, the robot immediately stops and waits for another random pose.
The reason for this package is to control a non-holonomic mobile robot in a Gazebo environment. The client can decide to make the robot begin moving or stop it. Whenever the client provides the order to begin moving, an arbitrary posture is picked and the robot begin moving. The client has the chances to stop the robot, regardless of whether an random posture has been now picked. At the point when the client provides the order to stop the robot, the robot quickly stops and waits for another random posture.


## To compile and run the package
After cloning the package, it is necessary to build the package in the path of your own workspace, with the command:
```
catkin_make
```
When the package is build effecively, you can run the .launch file to launch all the nodes and the Gazebo simulation:
```
roslaunch rt2_assignment1 sim.launch
```
## Description of the branch. 
In this organizer you can find:
action: contains the .action file Robotposition.action, needed to modify go_to_point as an action_server <--   
launch: having the launch file sim.launch, that starts the simulation in a Gazebo environment <--  
scripts: having two nodes implemented as python scripts that set the behaviour of the robot <--   
go_to_point: this is the action server that manage the robot speed control, depending on the goal received <--  
user_interface: sends the request to start/stop the go_to_point behaviour asking to the user what the robot needs to do <--   
src: contains two nodes implemented as cpp file that set the behaviour of the robot <--  
position_server: it is the server that generates a random position <--  
state_machine: makes the demand of a new goal and it send the request as a goal to go_to_point action server <--   

