# protoarm_bringup

This package houses all the main launch files needed to run different functionalities of the robot.

### `robot.launch`

### `sim.launch`

A launch file for running the robot arm in a gazebo simulation.

**Arguments**:
- `chess`: Runs a gazebo world with a chessboard 
  - **IMPORTANT:** Make sure that you add 
- `realsense`: Uses the `protoarm_realsense` URDF file that has a Realsense D345 RGB-D camera mounted on it

**IMPORTANT!!:** 

Make sure that you add the following export commands to your .bashrc so gazebo can find the models and realsense plugins respectively 
export GAZEBO_MODEL_PATH=/full/path/to/catkin_ws/src/arc_robot_arm/chessboard_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=/full/path/to/catkin_ws/devel/lib:$GAZEBO_PLUGIN_PATH
