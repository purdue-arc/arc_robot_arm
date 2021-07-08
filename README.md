# arc_robot_arm
This is where the robot arm project from the Autonomous Robotics Club at Purdue lives!!

## Quick start

1. [Setup ROS](http://localhost:4000/wiki/tutorials/setup-ros) if you haven't done so.

2. Clone this repo into the `src` folder in your ROS workspace
```
git clone https://github.com/purdue-arc/arc_robot_arm.git
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
conda env create -f robot_arm_environment.yml
conda activate robot-arm-env
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
![ik_demo](https://github.com/purdue-arc/wiki/blob/master/wiki/robot-arm/assets/gifs/ik_demo.gif)
