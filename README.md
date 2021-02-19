# arc_robot_arm
This is where the robot arm project from the Autonomous Robotics Club at Purdue lives!!

# Getting Started

## Prerequisites
1. Full ROS install of melodic or noetic on native linux/WSL or on virtual ROS env (i.e. [ROS Development Studio](https://rds.theconstructsim.com/))
2. ROS catkin workspace is setup (steps assume catkin workspace folder is `catkin_ws` and located in home dir
3. Clone this repository to `src` folder in catkin workspace 
4. Build (Compiles C/C++ files)
```
catkin build
```
or

```
cd ~/catkin_ws
catkin_make
```
5. Source  
```
source ~/catkin_ws/devel/setup.bash
```

## Quickstart (Sim or Sim + Real) 

### Sim Only
```
roslaunch arc_robot_arm robot.launch
rosrun arc_robot_arm test_kinematics
```
### Sim + Real

#### Prerequisites
- Arduino with arduino code loaded is connected at /dev/ttyACM0 (can change the launch file to match yours)
- Robot is wired + powered correctly, tested using Arduino test code
- Camera connected at device 0, is calibrated and has camera.yaml file loaded

#### Run Sim + Real 
```
roslaunch arc_robot_arm robot.launch real:=true
rosrun arc_robot_arm test_kinematics
```
#### Roslaunch params
- If no Arduino/Robot -> add `robot:=false` to roslaunch command
- If no Camera -> add `camera:=false` to roslaunch command
