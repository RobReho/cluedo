# Experimental robotic lab
First assignment for the Experimental Robotic Lab course a.y.2021/2022

## Introduction
At this stage of the project, the robot is a point that aims to explore its environment and deduce hypotheses based on hints it finds in different rooms. The robot has been designed to move randomly through the environment, entering different rooms and looking around for hints to make hypotheses. If a consistent hypothesis can be deduced, the robot will go to a designated location and express it in English. If the hypothesis is incorrect, the robot will continue exploring and finding new hints until a correct hypothesis is deduced.

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

The initialization is handled by the "cluedo_state_machine" node, which calls the appropriate services of the ARMOR server to make the ontology ready for the game and calls the service server /generate_murder to generate a winning hypothesis and store it for the following comparisons.
During the game, the "cluedo_state_machine" node asks for hints to the oracle by calling the service /get_hint, and it gets a hypothesis. Such hypothesis will be uploaded to the ontology, and the ARMOR server will be asked to reason with the new information and retrieve the classes "COMPLETE" and "INCONISTENT". If the hypothesis just uploaded is part of the class "COMPLETE" but not of the class "INCONISTENT", that means that it is a consistent hypothesis and can be queried to the Oracle. The "cluedo_state_machine" node does so by calling the service /verify_solution that will return 3 booleans for each element of the hypothesis (person, weapon, place). If all the booleans are true, the hypothesis is correct, and the game ends.

### State Machine
The different states implement with the Smach package are showed below.  

![Alt Text](https://github.com/RobReho/exproblab/blob/main/media/sm1.PNG)  

The states are implemented in the "cluedo_state_machine" node.
- The INIT state establishes communication with the Armor server, loads the OWL file, calls the "generate murder" service to start the game, and retrieves the list of people, weapons, and places from the OWL. 
- The EXPLORE state retrieves the list of available places, randomly chooses one, and simulates reaching the place by sleeping for 1 second.
- The MAKE HYPOTHESIS state asks the server to provide a new hint, loads it onto the ontology, and retrieves the classes "COMPLETE" and "INCONSISTENT" to check for consistency. If the hypothesis is consistent, the executed state is REACH ORACLE; otherwise, it goes back to the EXPLORE state.
- The REACH ORACLE state simulates reaching the oracle position by sleeping for 1 second. Although the possibility of this state failing is implemented in the state machine, it is never executed. The possibility is left for future implementations where an actual action will be implemented. Once the oracle is reached, the next executed state is DELIVER HYPOTHESIS.
- The DELIVER HYPOTHESIS state gets the person, weapon, and place of the hypothesis and expresses it in natural language. The next state is HYPOTHESIS CHECK.
- The HYPOTHESIS CHECK state calls the "verify solution" server to compare the hypothesis with the correct one. If all the booleans returned are true, the game ends; otherwise, the hypothesis is wrong, and the program executes the EXPLORE state.
## Installation and Running
This project needs some external packages. You can install them in your ROS workspace:  
ARMOR
```
  git clone https://github.com/EmaroLab/armor.git
```
SMASH
```
  git clone https://github.com/ros/executive_smach.git
  git clone https://github.com/ros-visualization/executive_smach_visualization.git
```
To install the package clone the repository in your ROS workspace:
```
  git clone https://github.com/RobReho/exproblab.git
```
then build your ROS workspace:
```
  catkin_make
```
To run the project launch the mani launch file:
```
  roslaunch cluedo launcher.launch
```
To visualize the Smash state machine graph, run in another tab:
```
  rosrun smach_viewer smach_viewer.py
```

## Demo

![Alt Text](https://github.com/RobReho/exproblab/blob/main/media/State_machine.gif)

## Working hypothesis and environment.
### System features
The game is a revisited simulated Cluedo game, where the player is the robot implemented by the state machine, and the game is controlled by the Oracle. The Oracle generates the hypothesis by choosing random elements in the people, weapons, and places arrays. Randomly, it might generate an inconsistent hypothesis, meaning that it will be composed of 4 elements instead of 3.
The robot will get both consistent and inconsistent hypotheses and send back only the consistent hypothesis to be compared with the solution. When a hypothesis is compared to the solution, the hints that don't match are discarded from the hints arrays stored in the Oracle node. As more hypotheses are compared, it becomes more and more likely that the proposed hypothesis matches the solution.
![Alt Text](https://github.com/RobReho/exproblab/blob/main/media/erl1_end.PNG)

### System limitations 
The game implemented has a very simple structure and does not use any IDs associated with the hypothesis. The architecture of the game has a very different way of generating and handling hypotheses compared to the following iterations. Nevertheless, the simplicity of the architecture makes it easy to adapt to future implementations.
### Possible technical Improvements
Possible improvements include a system that generates hints in a similar way to what happens in the following iterations.

## Contacts
Roberta Reho: s5075214@studenti.unige.it
