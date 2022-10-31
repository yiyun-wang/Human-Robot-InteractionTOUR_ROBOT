
# TOUR_ROBOT 🤖

INFO 5356-030: Introduction to Human-Robot Interaction 

Lab 6 - Tour Robot

In this lab, you will learn how to design a robot that socially navigates with expressive behaviors as it escorts new visitors to their destination. Building on the Social Navigation Node, you will create a state machine to generate the expressive robot behavior and control architecture, and test the robot with a user.

The learning outcomes of Lab 6 are:
- [Task 1 - Create a State Machine](#Task-1---Create-a-State-Machine)
- [Task 2 - Define a Robot Control Architecture](#Task-2---Define-a-Robot-Control-Architecture)
- [Task 3 - Test Expressive Navigation Node on the Turtlebot](#Task-3---Test-Expressive-Navigation-Node-on-the-Turtlebot)

## Prerequisite

### Download Tour Robot package
``` 
cd ~/<ros_workspace>/src/
git clone https://github.com/Cornell-Tech-Intro-HRI/tour_robot
mv tour_robot_pkg/run_tour_robot.sh ~/<ros_workspace>/
```

## Task 1 - Create a State Machine

The goal of this task is to build a state machine in the update_state_machine function to generate simple expressive behavior for the robot including a ‘robot-dance’ gesture, displaying lightring sequences, or both. 

1. Navigate from the docking station to a user situated at a robot waypoint. 
2. The robot greets a user, generates an expressive behavior, and asks the user for their destination.
3. The robot escorts a user from the participant’s initial position to their destination.
4. The robot stops, generates an expressive behavior, alerts a user that they have reached their destination, and says goodbye to the user.
5. The robot navigates to the docking station and the state machine ends.


Each state needs a condition to advance to the next state. 

The tour_robot_node.py node includes a helper functions to get started:
- generate_gesture_expression - This function sends motion commands to the robot in terms of linear velocity (x-direction) and angular velocity (x-direction) for the robot to rotate right and then rotate left.
- generate_lightring_expression - This function sends commands to the robot’s lighting so that it generates a lightright sequence.
- generate_gesture_lightring_expression - This function calls generate_gesture_expression and 
- generate_lightring_expression to generate both gesture and lightring expressions.
- robot_talker - Uses text to speech software to enable the robot to alert users when they are in the intimate and public zones using a given robot phrase and output .mp3 filename.
- move_robot - This is a helper function for the generate_gesture_expression function by sending motion commands to the robot in terms of linear velocity (x-direction) and angular velocity (x-direction).
- add_goal - add waypoint goal for the robot to move from its current position to an input x-, y- directions, and rotation.

### Step 1: Customize package.xml Update the following fields

- Maintainer - Enter group member names
- Maintainer Email - Email one email address from the student group members
- Description - Add a brief description of the proxemic detector package

### Step 2: Write update_state_machine function. 

Open the file located at:
```
cd ~/<ros_workspace>/tour_robot_pkg/tour_robot_pkg/tour_robot_node.py
```

### Step 3: Run the tour_robot node. 

Open a terminal and run:
```
bash ~/<ros_workspace>/run_tour_robot.sh
```

### Checkpoint 1: Write update_state_machine function.

Your task is to write the update_state_machine function which is responsible for updating the state machine based on the robot’s actions. States describe simple actions the robot performs e.g., approach user, give tour. The robot’s state is updated using a transition that may require a condition to exit that state e.g., s1 = approach, s2 = ‘give tour’, condition is that the robot completes all navigation and expressive behavior before proceeding to the next state. 

The TourRobot Class includes state variables to keep track of the robot’s state over the course of an interaction (i.e., state1, state2, etc). Feel free to modify the state variables and variable names.


## Task 2 - Define a Robot Control Architecture

The goal of this task is to design a robot control architecture for your robot which could be either deliberative, reactive, and hybrid.

- Deliberate architecture uses the classic ‘sense-plan-act’ paradigm. The sensor data is used to plan and generate actions for the robot to perform to its actuators. The plan is executed without using the sensor that created the model. 

- Reactive architecture (also known as the subsumption architecture) is built from layers of finite state machines where each state connects sensors to actuators. Each finite machine describes a behavior which is often called behavior-based architecture. 

- Hybrid architecture is a combination of the deliberate and reactive. The lower level behaviors provide reactive behaviors and the higher level behaviors provide deliberate behaviors.

### Checkpoint 2: Define a robot control architecture for the robot. 

Identify and describe the architecture your team chose for this lab assignment along with justification for its design. Include a figure that shows the control architecture including the sensor data, robot’s actions, and that connects to the robot’s motors for motion control (see slides on Canvas for examples).

## Task 3: Test Expressive Navigation Node on the Turtlebot. 

The tour robot node is expected to escort a user from their starting location to their destination while generating expressive behavior while speaking to the user. 

### Checkpoint 3: Record Videos of robot expressions.

Your task is to test your node with a user for three expressive behaviors including ‘robot-dance’ gesture, lightring (sequence), and both. Record three videos, one for each expression.

## Further Issues and questions ❓ 

If you have issues or questions, don't hesitate to contact [Angelique Taylor](https://www.angeliquemtaylor.com/) at amt298@cornell.edu or [Tauhid Tanjim](https://tanjim13.github.io/tauhidtanjim/) at tt485@cornell.edu.