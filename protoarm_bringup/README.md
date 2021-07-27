# protoarm_bringup

This package houses all the main launch files needed to run different functionalities of the robot.

## Quick Start 

> Make sure to [setup](https://github.com/purdue-arc/arc_robot_arm/blob/main/README.md) your robot arm environment if you haven't already

### Base Gazebo world
```
roslaunch protoarm_bringup sim.launch
```

### With chessboard and chess pieces world
1. Add the following export commands to your `~/.bashrc` (or ~/.zshrc) so gazebo can find the chess models
```
export GAZEBO_MODEL_PATH=/EDIT_ME/full/path/to/catkin_ws/src/arc_robot_arm/chessboard_gazebo/models:$GAZEBO_MODEL_PATH
```
2. Launch the chessboard world
```
roslaunch protoarm_bringup sim.launch chess:=true
```

### With the Gazebo realsense camera 
1. Add the following export commands to your `~/.bashrc` (or ~/.zshrc) so gazebo can find the realsense plugins
```
export GAZEBO_PLUGIN_PATH=/EDIT_ME/full/path/to/catkin_ws/devel/lib:$GAZEBO_PLUGIN_PATH
```
2. Launch the chessboard world
```
roslaunch protoarm_bringup sim.launch realsense:=true
```
3. Run `rqt` in another window (make sure to activate conda env and source ROS workspace in the new env if not done automatically)
4. In the new window, click on `Plugins > Visualization > Image View` 
5. Select the `/camera/color/image_raw` topic and see the camera output 

## Documentation

### `robot.launch`

A launch file for running the robot arm in real life. 

TODO: New packages aren't fully tested with the real robot arm

### `sim.launch`

A launch file for running the robot arm in a gazebo simulation.

**Arguments**:
- `chess`: Runs a gazebo world with a chessboard 
- `realsense`: Uses the `protoarm_realsense` URDF file that has a Realsense D345 RGB-D camera mounted on it
- `verbose`: Gives gazebo verbose output

## Debugging
- Running `rostopic list` is a good sanity check. You should see the controller topics, moveit topics, camera topics if you are running the realsense camera, etc 
- For Gazebo, run with `verbose` to get more Gazebo messages that would be hidden otherwise. Most commonly it can't find paths to things, which need to be specified with `GAZEBO_MODEL_PATH` or `GAZEBO_PLUGIN_PATH`
- Use Google obviously
