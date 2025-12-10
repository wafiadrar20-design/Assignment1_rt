This project implements the requirements of the first assignment using ROS2 and the turtlesim simulator. The project includes two main nodes: one node to control the two turtles using a text interface, and another node to calculate the distance between them and check the boundaries.

First: What has been done?
	1.	Creating a new Workspace
A new workspace was created for the project using the following commands:
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
	2.	Creating a new package named assignment1_rt
The package was created using Python, and the needed dependencies were added:
ros2 pkg create –build-type ament_python assignment1_rt –dependencies rclpy std_msgs geometry_msgs turtlesim
Then a folder for the nodes was created inside the package, and the following files were added:
ui_node.py
distance_node.py
	3.	UI Node
This node provides a simple text interface that allows the user to choose which turtle to control (turtle1 or turtle2) and to enter the linear and angular speed.
The node sends the speed for only one second and then stops the robot.
It also publishes the name of the moving turtle on the topic /moving_turtle.
	4.	Distance Node
This node subscribes to the pose topics of turtle1 and turtle2.
It calculates the distance between the two turtles and publishes it on the topic /turtles_distance.
The node stops the robot if the distance becomes smaller than a specific limit.
It also checks the map boundaries and stops the robot if it goes outside the allowed range (less than 1.0 or more than 10.0).
	5.	A problem I faced during development
I had an issue in the setup.py file related to the entry points.
The names of the nodes were not registered correctly, and ROS2 could not run them with the command ros2 run.
I fixed the entry points by adding the correct node names to the console_scripts section, and after that ROS2 was able to run both nodes without problems.

Second: How to run the project
	1.	Start the turtlesim simulator
ros2 run turtlesim turtlesim_node
	2.	Spawn the second turtle
ros2 service call /spawn turtlesim/srv/Spawn “{x: 5.0, y: 5.0, theta: 0.0, name: ‘turtle2’}”
	3.	Build the project
cd ~/ros2_ws
colcon build
source install/setup.bash
	4.	Run the distance node
ros2 run assignment1_rt distance_node
	5.	Run the UI node
ros2 run assignment1_rt ui_node

After running the nodes, the user can control the selected turtle, and the speed is sent for only one second. The distance node protects the turtles from collision and from going outside the map boundaries.
