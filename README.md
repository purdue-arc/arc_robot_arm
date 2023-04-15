# arc_robot_arm

Welcome to the ARC Robot Arm project repo!

Our overarching goal is to explore the robotic manipulation by building a chess-playing robot arm.

## Quick start 

1. Setup ROS Noetic if you haven't done so.


2. Clone this repo into the `src` folder in your ROS workspace recursively to clone dependencies 

```
git clone --recursive https://github.com/purdue-arc/arc_robot_arm.git
```

3. Download all ROS package dependencies / Setup environment

```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
```

4. Build + source (Do this every time you download new packages)
```
catkin build
source path/to/catkin_ws/devel/setup.bash
```
> Note: Build + source every time you add new packages. Source every time you open a fresh terminal, or add the line to your ~/.bashrc so it sources automatically

5. Run robot with rviz to display joint state and overhead camera

```
roslaunch arm_launch robot.launch rviz:=true overhead:=true display_overhead:=true
```
> Now, you should see an image view from the camera feed and robot in RViz

## Grasping with RelaxedIK

1. [Install Rust](https://www.rust-lang.org/tools/install)
2. Revert rust toolchain to supported version

```
rustup install 1.58.1 && rustup default 1.58.1 
```
3. Build rust code
```
roscd relaxed_ik_ros1/relaxed_ik_core && cargo build
```

5. Run grasping demo 

```
roslaunch arm_launch grasping.launch
```

### Sources

[RelaxedIK code](https://github.com/uwgraphics/relaxed_ik_core)

