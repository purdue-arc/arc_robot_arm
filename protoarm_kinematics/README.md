# protoarm_kinematics

This package houses the MoveIt wrapper that we can interface with to move the arm to joint and goal positions.

### Quick Start

1. Setup environment in the [main README](https://github.com/purdue-arc/arc_robot_arm/blob/main/README.md)

2. Launch the robot in sim
```
roslaunch protoarm_bringup sim.launch
```
3. Run some test goal positions (in another terminal window)
```
roscd protoarm_kinematics/src
rosrun protoarm_kinematics test_kinematics
```
Should see the robot move something like this (to the right and back to the left) in Gazebo and MoveIt:
![ik_demo](https://github.com/purdue-arc/arc_robot_arm/blob/main/assets/gifs/ik_demo.gif)

### `move_to`

A ROS node with two subscribers that listen for goal joint state msgs or goal pose msgs and then interface with the Kinematics library which follows through with the planning and execution of the move request.

**Subscribers**:
- `move_to_joint_state`: Listens for [JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html) msgs that have the correct number of joints as the robot arm
- `move_to_pose`: Listens for [PoseStamped] msgs. 
  - Note: For the protoarm, only xyz coordinates are considered, orientation is not considered since the arm is 5DOF, and planning commonly fails with orientation with the KDL inverse kinematics planner that we use. For more information, see the [MoveIt docs](https://ros-planning.github.io/moveit_tutorials/doc/kinematics_configuration/kinematics_configuration_tutorial.html#position-only-ik).

TODO: Using subscribers and publishers for planning + execution isn't ideal as it should be done with ROS services instead for more robustness. But pubs and subs work for now.
