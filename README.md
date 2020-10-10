# arc_robot_arm
This is where the robot arm project from the Autonomous Robotics Club at Purdue lives!!

### General Information
- ROS version: Melodic
- TODO

# Getting Started

## Get Project Workspace Setup on ROS Development Studio

1. Sign in/Sign up at https://rds.theconstructsim.com/ to get your own ROS Dev workspace
2. Follow this link to "fork" (make your own copy) of the ROSject
3. Click the red "Open" button to load the ROSject
4. Once the workspace loads (can take a few min), that's it! Your workspace is set up!

## Get Familiar with Dev Environment

### Terminal

- Click on "Tools" in the top left corner, then click "Terminal"

### IDE: VSCode

- Click on "Tools" in the top left corner, then click "IDE"

### Rviz

- Click on "Tools" in the top left corner, then click "Graphical Tools"
- NOTE: you must start rviz by using the "roslaunch" command on a rviz enabled launch file or to just open Rviz, otherwise nothing will load here

### RDS File Structure
```
user/ <-- You should be here on Terminal load
│
└───ai_ws/
│
└───catkin_ws/
│
└───datasets_ws/
│
└───notebook_ws/
│
└───webpage_ws/
│
└───simulation_ws/
    │
    └───src/
        │
        └───arc_robot_arm/
           │
           └───arc_robot_arm/ (main project folder with all the nodes + launch files for the arm)
               │
               └───nodes/ (ROS Python scripts with nodes for path planning + ROS/Arduino Interface, etc)
               │
               └───launch/ (Launch files)
           │
           └───arc_robot_arm_moveit_config/ (ARC robot arm MoveIt config)
           │
           └───robot_arm_urdf_description/ (URDF description of ARC robot arm)
```
Play around with more of the features! RDS/The Construct has tutorials on YouTube!
           

## Setup GitHub to push,pull code

1. TODO
