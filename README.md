# arc_robot_arm

Welcome to the ARC Robot Arm project repo!

Our overarching goal is to explore the robotic manipulation, robot vision, robot control, and reinforcement learning by building a robot arm. In our journey, we plan to publish our progress, tutorials, and understandings. Learn more on the [wiki](https://wiki.purduearc.com/wiki/robot-arm/start-here).

## Quick start

1. [Setup ROS](http://localhost:4000/wiki/tutorials/setup-ros) if you haven't done so.

2. Clone this repo into the `src` folder in your ROS workspace recursively to get the `yolov5_pytorch_ros` and `realsense_ros_gazebo` packages
```
git clone --recursive https://github.com/purdue-arc/arc_robot_arm.git
```
3. Download all package dependencies / Setup environment

**Non-Robostack**:
```
cd path/to/root_workspace_dir
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
```
**Robostack Only**:
```
# Ensure base conda environment is activated
cd arc_robot_arm
mamba env create -f robot_arm_environment.yml
conda activate robot-arm-env
```
4. Build + source
```
catkin build && source path/to/catkin_ws/devel/setup.bash
```
5. Launch the robot in sim
```
roslaunch protoarm_bringup sim.launch
```
6. Run some test goal positions (in another terminal window)
```
roscd protoarm_kinematics/src && rosrun protoarm_kinematics test_kinematics
```
Should see the robot move something like this (to the right and back to the left) in Gazebo and MoveIt:
![ik_demo](https://github.com/purdue-arc/arc_robot_arm/blob/main/assets/gifs/ik_demo.gif)

## What next?

1. Check out the wiki pages [here](https://wiki.purduearc.com/wiki/robot-arm/start-here) to learn more about all the different components and how they fit together.
2. Follow links in the wiki to learn about a component you're interested in, check out related ROS packages and read the code. 
