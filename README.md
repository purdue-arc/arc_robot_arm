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

5. 
    ### Terminal/Workspace Setup

    - Click on "Tools" in the top left corner, then click "Terminal"
    - As shown in the file structure below, there are multiple workspaces(technically _catkin_ workspaces). We will only be using the simulation_ws for now.
    - Make sure to **ALWAYS** source the devel/setup.sh for the catkin workspace whenever the ROSject loads:
        -`source devel/setup.sh` (executed when in simulation_ws directory)

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
            └───arc_robot_arm/ (Name of GitHub repo)
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
           
6. 
    ## Setup GitHub to push,pull code
    1. Navigate to the `arc_robot_arm/ (Name of GitHub repo)` directory shown in the above file structure
    2. Run `git status`. If you don't get any errors, that means all has gone well so far.
    3. Setup your own branch to safely make changes to using this command: `git checkout -b <branch>`
        - NOTE: the name of the branch should be something logical related what you are changing
    4. Make some changes
    5. Stage + commit your changes to your branch in one command: `git commit -am "Message for my commit"`

## Get Familiar with Dev Environment

### IDE: VSCode

- Click on "Tools" in the top left corner, then click "IDE"

### Rviz

- Click on "Tools" in the top left corner, then click "Graphical Tools"
- NOTE: you must start rviz by using the "roslaunch" command on a rviz enabled launch file or to just open Rviz, otherwise nothing will load here

Play around with more of the features! RDS/The Construct has tutorials on YouTube!
