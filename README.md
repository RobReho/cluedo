# Experimental robotic lab
First assignment for the Experimental Robotic Lab course a.y.2021/2022

## Introduction
At this stage of the project the robot is a point that aims at exploring its environment and deducing hypotheses based on hints it finds in different rooms. The robot has been designed to move randomly through the environment, entering different rooms and looking around for hints to make hypotheses. If a consistent hypothesis can be deduced, the robot will go to a designated location and express it in English. If the hypothesis is incorrect, the robot will continue exploring and finding new hints until a correct hypothesis is deduced.

## Software architecture

### ROS nodes
The nodes in this package are:
- "cluedo_state_machine": handles the communication with the ARMOR server.
- "oracle": controls the hints generation and holds the solution of the game.
- "user_interface": subscribes to the other node's messages and prints the on the terminal. The node is not represented in the following diagrams for sake of semplicity.

### ROS services
The nodes communicate with some customized services:
- "/verify_solution" of type Compare
- '/generate_murder' of type Hypothesis
- '/get_hint' of type Hints

### Temporal Diagram
In the following temporal diagram is showed he communication between the nodes.
![Alt Text](https://github.com/RobReho/exproblab/blob/main/media/erl1_temp.PNG)  
The initialization is handeled by the "cluedo_state_machine" node, that calls the appropriate services of the ARMOR server to make the ontology ready for the game,  
and calls the service server /generate_murder to generate a winning hypothesis and store it for the following comparisons.  
During the game, the "cluedo_state_machine" node asks for hints to the oracle calling the service /get_hint and it get an hypothesis. Such hypothesis will be uploaded on the ontology and the ARMOR sever will be asked to reason with the new information and retireve the classes "COMPLETE" and "INCONISTENT". If the hypothesis just uploaded is part of the class "COMPLETE" but not of the class "INCONISTENT", that means that it is a consistent hypothesis and can be queried to the Oracle. The "cluedo_state_machine" node does so by calling the service /verify_solution that will return 3 booleans for each element oof the hypothesis (person, weapone, place).
If all the booleans are true the hypothesys is correc and the game ends.

### State Machine
The different states implement with the Smach package are showed below.
![Alt Text](https://github.com/RobReho/exproblab/blob/main/media/sm1.PNG)
The states are implemented in the "cluedo_state_machine" node.
- The INIT state Establish the communication with Armor server, loads OWL file, calls "generate murder" service to start the game, retrieves people, weapons and places list from the OWL 
- The EXPLORE state retrieves the list of available places, randomly choose one and simulates reaching the place by sleeping 1 second.
- THe MAKE HYPOTHESIS state asks the server to get a new hint, it loads it on the ontology and retrieves the classes "COMPLETE" and "INCONISTENT" to check for consistency. If the hypothesis is consistent the executed state is REACH ORACLE, otherwise it will go back to the state EXPLORE.
- The state REACH ORACLE just simulates reaching the oracle posistion by sleeping 1 second. The possibility that this state fails is implemented in the state machine, but never executed. The possibility is left for future implementations where an actual sction will be implemented. Onche the oracle is reached, the next executed state is DELIVER HYPOTHESIS.
- The DELIVER HYPOTHESIS state gets the person, weapon and place of the hypothesis and express it in natural language. The next state is HYPOTHESIS CHECK
- The state HYPOTHESIS CHECK calls the server "verify solution" to compare the hypothesis with the right one. If all the booleans returned are true the game ends, otherwise the hypothesis is wrong and the program executes the state EXPLORE.

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

![Alt Text](https://github.com/RobReho/exproblab/blob/main/media/State_machine.gif)

## Working hypothesis and environment.
### System features
The game
2. System limitations (1 or 2 paragraph).
3. Possible technical Improvements (1 or 2 paragraph).

## Contacts
Roberta Reho: s5075214@studenti.unige.it
