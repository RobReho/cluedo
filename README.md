# Experimental robotic lab
First assignment for the Experimental Robotic Lab course a.y.2021/2022

## Introduction
At this stage of the project the robot is a point that aims at exploring its environment and deducing hypotheses based on hints it finds in different rooms. The robot has been designed to move randomly through the environment, entering different rooms and looking around for hints to make hypotheses. If a consistent hypothesis can be deduced, the robot will go to a designated location and express it in English. If the hypothesis is incorrect, the robot will continue exploring and finding new hints until a correct hypothesis is deduced.

## Software architecture
![Alt Text](https://github.com/RobReho/exproblab/blob/main/media/erl1_temp.PNG)  

![Alt Text](https://github.com/RobReho/exproblab/blob/main/media/sm1.PNG)

temporal diagram and states diagrams (if
applicable). Each diagram should be commented with a paragraph,
plus a list describing ROS messages and parameters.

## Installation and Running
This project needs some external packages. You can install them in your ROS workspace:  
ARMOR
'''
  git clone https://github.com/EmaroLab/armor.git
'''
SMASH
'''
  git clone https://github.com/ros/executive_smach.git
  git clone https://github.com/ros-visualization/executive_smach_visualization.git
'''
To install the package clone the repository in your ROS workspace:
'''
  git clone https://github.com/RobReho/exproblab.git
'''
then build your ROS workspace:
'''
  catkin_make
 '''
To run the project launch the mani launch file:
'''
  roslaunch cluedo launcher.launch
'''
To visualize the Smash state machine graph, run in another tab:
'''
  rosrun smach_viewer smach_viewer.py
'''

## Demo
A commented small video, a GIF or screenshots showing the relevant
parts of the running code.  

![Alt Text](https://github.com/RobReho/exproblab/blob/main/media/State_machine.gif)

## Working hypothesis and environment (1 or 2 paragraph).
1. System�s features (1 or 2 paragraph).
2. System�s limitations (1 or 2 paragraph).
3. Possible technical Improvements (1 or 2 paragraph).

## Contacts
