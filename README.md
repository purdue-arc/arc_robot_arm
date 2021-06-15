# arc_robot_arm

## Goal
With this project, our goal is to explore the robotic manipulation, robot vision, robot control, and reinforcement learning by building a robot arm. In our journey, we plan to publish our progress, tutorials, and understandings [here](wiki.purduearc.com) - WIP as of 6/14/21.

## Protoarm
This is protoarm (short for prototype arm), a 5-DOF robot arm adapted slightly from [HowToMechatronics' model](https://www.youtube.com/watch?v=_B3gWd3A_SI) that we built first to understand ROS, MoveIt, and the software stack for robot arms.
![protoarm](https://github.com/purdue-arc/arc_robot_arm/blob/main/assets/images/protoarm.png)

## Quick start

1. [Install ROS](https://wiki.purduearc.com/wiki/tutorials/setup-ros)

2. Clone this repo into the `src` folder in your ROS workspace
```
git clone https://github.com/purdue-arc/arc_robot_arm.git
```
3. Download all package dependencies
```
cd path/to/workspace
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
```
4. Build + source
```
catkin build && source path/to/catkin_ws/devel/setup.bash
```
5. Launch the robot
```
roslaunch protoarm_bringup robot.launch
```
6. Run some test goal positions (in another terminal window)
```
roscd protoarm_kinematics/src && rosrun protoarm_kinematics test_kinematics
```
Should see the robot move something like this (to the right and back to the left) in Gazebo and MoveIt:
![ik_demo](https://github.com/purdue-arc/arc_robot_arm/blob/main/assets/gifs/ik_demo.gif)
