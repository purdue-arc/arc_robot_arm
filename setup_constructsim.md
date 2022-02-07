## Set up arc_robot_arm code using the ConstructSim 

### Quick start

1. [Create](https://app.theconstructsim.com) a ConstructSim account if you haven't already here 

2. Fork [this ROSject](https://app.theconstructsim.com/#/Rosject/459280) (If you cannot find it, search for `ARC Robot Arm` > hit fork on the first result)

3. Run the ROSject that you just forked 

4. Open the web shell (leftmost button on the bottom)

5. Update repo, build, source 

```bash
cd ~/catkin_ws/src/arc_robot_arm
git pull
```

```bash
# Installs all dependencies
cd ~/catkin_ws # must be in workspace root dir
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
```

```bash
# Builds all your ROS packages
catkin build 
```
> If you get the following error `Killed signal terminated program cc1plus` where the build crashes for the `realsense_gazebo_ros` pkg, run this to limit the memory usage by the build command: `catkin build --force-cmake --mem-limit 95`

```bash
# Sets your workspace dir + updates package links
source ~/catkin_ws/devel/setup.bash 

# Optional, but convenient. Sources your workspace every time you open a new shell by adding it to ~/.bashrc
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc 
```

> Note: Build + source every time you add new packages. Source every time you open a fresh terminal, or add the line to your ~/.bashrc so it sources automatically

6. Continue following tutorials in the ROS packages
- Launch the robot in Gazebo simulation with the chessboard world and Realsense ROS gazebo camera, using the [protoarm_bringup](https://github.com/purdue-arc/arc_robot_arm/tree/main/protoarm_bringup) package 
> Can skip the `export GAZEBO..` steps as they are already done for you
- Use the IK to run some test goal positions using the [protoarm_kinematics](https://github.com/purdue-arc/arc_robot_arm/tree/main/protoarm_kinematics) package 


## Questions or problems?

Add an [issue](https://github.com/purdue-arc/arc_robot_arm/issues/new/choose) or add comments under articles in the wiki and we'll respond ASAP!