# Research Track -2 Assignment-1

Koushikmani Maskalmatti Lakshman (S5053566)
## Package explanation

The package manages the mobile non-holonomic robot with a simple 'go_to_point' behaviour:

*An irregular objective is given (a posture, [x,y,theta]) <--  
*The robot situates itself towards the [x,y] destination<--  
*Then, at that point, drives directly to that position (changing the direction assuming need)<--  
*Having reached the [x,y] objective position the robot turns set up to match the objective theta<--  
*In the event that the client doesn't stop the robot GOTO stage 1, in any case stay still until requested to begin once more, then, at that point, GOTO stage <--  

Since the client demand is here carried out as an action it tends to be acquired, stoppinng the robot whenever and afterward restarting it while giving another objective.

## Content description

Two nodes are carried out as python scripts

go_to_point.py: the action server dealing with the robot speed control contingent upon the objective goal.
user_interface.py: the simple command line UI, which sends the request to begin/stop the go_to_point behaviour.

While the last two are C++ based nodes

position_service.cpp: the server creating an arbitrary posture [x,y,theta] as a reaction to a request.
state_machine.cpp: the FSM dealing with the request of another objective posture while required, sending it as an objective to 'go_to_point' action server.

At last, the control can be applied to a robot recreated utilizing Coppeliasim (see Requirements), for which two scenes are here introduced

pioneer_scene.ttt: a basic scene with a Pioneer p3dx non-holonomic versatile robot in a vacant environment.

robotnik_scene.ttt: a simple scene scene with a Robotnik Summit XL140701 non-holonomic mobile robot in a vacant environment.

## To Run and compile

Compilation can be done as always with

               path/to/ros_ws/$ catkin_make

Two launch files are given

sim.launch: to be utilized in order to launch all the nodes and the Gazebo simulation

               path/to/ros_ws/$ roslaunch rt2_assignment1 sim.launch

For this situation the Gazebo recreation will naturally begin.

sim_coppelia.launch: to be utilized in order to all the nodes which will gives with the Coppelia simulation.

               path/to/ros_ws/$ roslaunch rt2_assignment1 sim_coppelia.launch

For this situation CoppeliaSim should be begun independently (make sure to have a case of roscore running prior to sending off the CoppeliaSim executable). The reproduction can be either begun previously or subsequent to sending off then nodes, yet don't attempt to run another recreation when the hubs have been running on a past one (or the framework could wind up in an underlying state differente from the expected to be one, always being unable to arrive at the goal).In different terms, each time  simulation is restarted the nodes should be to, and vice-versa (generally).

## Notebook

A Jupyter Notebook can be found under the notebooks folder. By executing it the robot can be controlled with a graphical UI giving additionally ongoing charts of the robot conduct. More subtleties in the actual journal. To begin the journal execute

             path/to/ros_ws/src/rt2_assignment1/notebooks$ jupyter notebook --allow-root --ip 0.0.0.0
             
Then open web browser at localhost:8888 and select the notebook           

## Execution description

## Finite StateMachine

The main decision worth of note likely respects the way that the current robot state can be changed by either the client's feedback (1: begin, - 1: stop) or the action arriving at its objective goal (2: activity finished): in the last option case the condition of the objective goal is recovered, and a check is made on regardless of whether the activity was succesful. On the off chance that it succeeded, it begins again by characterizing another irregular objective point, if not the robot will pause and sit tight for new client inputs.


## Prerequisites

Gazebo is expected to run the first launch file (the scene definition is introduced in this package). Coppeliasim is expected to run the second launch file http://www.coppeliarobotics.com/downloads.html 

## Documentation

Next to this README further documentation of all classes and techniques can be found in the doc organizer.


## Impediments and issues

Assuming you take a stab at running both the Gazebo and CoppeliaSim and the last option appears to not answer to the hubs, while the UI results frozen subsequent to having advised the framework to run, attempt to kill the roscore interaction; this may be connected with Gazebo overwriting a few qualities connected with the simulation (most likely reproduction time) and these not being properly "cleaned" whenever Gazebo is shut.


