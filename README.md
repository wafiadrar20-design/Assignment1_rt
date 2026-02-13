ROS 2 Turtlesim Control and Safety System (Assignment 1)

Author: Amri Abdelouafi
Student ID: 7708121
ROS 2 Distro: Jazzy
Simulator: turtlesim

⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻


1. Project Overview

This project implements the requirements of Assignment 1 using ROS 2 and the turtlesim simulator.

The system consists of two main nodes:

	•	A User Interface Node that allows manual control of two turtles.
	•	A Distance & Safety Node that monitors the distance between the turtles and enforces safety constraints.

The objective is to:

	•	Control either turtle using a text-based interface.
	•	Continuously compute the distance between the two turtles.
	•	Prevent collisions.
	•	Ensure turtles stay within valid map boundaries.

⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻


2. System Architecture

2.1 Workspace

The project was developed inside a dedicated ROS 2 workspace:

    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    colcon build
    source install/setup.bash

2.2 Package

Package name: assignment1_rt

Created using:

     ros2 pkg create --build-type ament_python assignment1_rt --dependencies rclpy std_msgs geometry_msgs turtlesim

Inside the package, two main Python nodes were implemented:

	•	ui_node.py
	•	distance_node.py

⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻

3. Nodes Description

3.1 UI Node (ui_node.py)

This node provides a simple text-based interface.

Features:

	•	Allows the user to select:
	•	turtle1
	•	turtle2
	•	Accepts:
	•	Linear velocity
	•	Angular velocity
	•	Publishes velocity commands for one second only
	•	Automatically stops the turtle after one second
	•	Publishes the name of the moving turtle on:

    /moving_turtle


⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻

3.2 Distance Node (distance_node.py)

This node implements safety monitoring.

Subscriptions:

	•	/turtle1/pose
	•	/turtle2/pose

Publications:

	•	/turtles_distance (calculated distance between turtles)

Safety Logic:

	1.	Collision Prevention
	•	Computes Euclidean distance between turtles.
	•	If the distance becomes smaller than a defined threshold,
	
the moving turtle is stopped immediately.

	2.	Boundary Protection
	•	The node checks map limits.
	•	If the turtle position is:
	•	Less than 1.0
	•	Greater than 10.0
	
The turtle is stopped automatically.

This guarantees:

	•	No collision between turtles.
	•	No movement outside allowed map boundaries.

⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻

4. Development Issue and Solution

During development, an issue occurred in the setup.py file related to entry points.

The node names were not correctly registered inside the console_scripts section, which prevented execution using:

    ros2 run assignment1_rt <node_name>

⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻

5. How to Run the Project

Step 1 – Start turtlesim

     ros2 run turtlesim turtlesim_node

Step 2 – Spawn the Second Turtle

     ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"

Step 3 – Build the Project

    cd ~/ros2_ws
    colcon build
    source install/setup.bash

Step 4 – Run the Distance Node

     ros2 run assignment1_rt distance_node

Step 5 – Run the UI Node

     ros2 run assignment1_rt ui_node

⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻⸻

6. Final Behavior

After launching both nodes:

	•	The user can select which turtle to control.
	•	Motion commands are applied for only one second.
	•	The system continuously monitors:
	•	Inter-turtle distance.
	•	Map boundaries.
	•	The turtles are automatically stopped if:
	•	They get too close.
	•	They exit the allowed map range.
