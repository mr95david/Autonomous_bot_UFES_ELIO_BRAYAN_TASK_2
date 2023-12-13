# Autonomous_bot_UFES_ELIO_BRAYAN_TASK_2
## Package Description

The following package contains files related to the launch for the activities requested in Activity 2 of the Autonomous Robots class at the Federal University of Espirito Santo. We continue with the main requirements for the package's operation.

## Software Requirements
The package was built with:
- Ubuntu Debian 22.04.3 LTS
- Using ROS2 (Robot Operating System)
- Using the Humble Hawksbill version
- Nav2 - version - 1.0.0

Continuing with the necessary packages, the package was mainly built in Python 3.12.

## Necessary Libraries
- Turtlebot3 for ROS2 humble and the packages installed in the Nav navigation tutorial.
- [ROS2 Nav2 - Navigation Stack](https://www.youtube.com/watch?v=idQb2pB-h2Q&t=1320s)

## Package Content
The package contains:
- 3 .world file models comprising the robot simulation worlds:
  - mundo_final.world
- Described in the following images.

## Execution Commands
The package was designed to run with the following execution commands:
- `ros2 launch map_launch launch_mapa.launch.py`
- `ros2 launch map_launch launch_prueba.launch.py`
- `ros2 launch map_launch launch_robot.launch.py`

This will execute the nodes of the navigation package simulation and the object detection storage package. In addition, we include a possible package of Python requirements.

## Python Requirements Package
The Python requirements package includes:
- Python 3.12 libraries
- ROS2 packages and dependencies
- Turtlebot3 package dependencies
- YOLO v8

This ensures the correct operation of the nodes and the simulation environment.

## Additional Notes
- The Turtlebot3 model is used for the robot simulations.
- The `mundo_final.world` files represent different simulation environments.
- It is crucial to follow the Nav navigation tutorial for successful setup and operation.
- The YouTube tutorial link provides a quick guide on setting up and using the ROS2 Nav2 Navigation Stack.

## Credits and Acknowledgments
- Professor: Ricardo Carminati de Mello
- Course: Autonomous Robots
- Institution: Federal University of Espirito Santo

## Contact Information
For further queries or support, contact:
- Email: eliotrianar95@gmail.com
- Phone: (27) 996 561 995
- Address: Federal University of Espirito Santo
