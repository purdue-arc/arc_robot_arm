# arc_robot_arm
This is where the robot arm project from the Autonomous Robotics Club at Purdue lives!!

### General Information
- ROS version: Melodic
- TODO

# Getting Started

## Get Project Workspace Setup on ROS Development Studio

1. Sign in/Sign up on the [ROS Development Studio](https://rds.theconstructsim.com/) to get your own ROS Dev workspace
2. Follow this [link](http://www.rosject.io/l/1762f8b9/) to "fork" (make your own copy) of the ROSject
3. Click the red "Open" button to load the ROSject
4. Once the workspace loads (can take a few min), that's it! Your ROS workspace is created!

### Terminal/Workspace Setup

1. Click on "Tools" in the top left corner, then click "Terminal" for the terminal to load
 NOTE:
    - As shown in the file structure below, there are multiple workspaces(technically _catkin_ workspaces). We will only be using the simulation_ws for now.
    - Make sure to **ALWAYS** source the devel/setup.sh for the simulation_ws or whatever workspace you're in whenever the ROSject loads to let know ROS what           workspace you are using: `source devel/setup.sh` (executed when in simulation_ws directory)

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
## Setup GitHub to push,pull code
1. Navigate to the `arc_robot_arm/ (Name of GitHub repo)` directory shown in the above file structure
2. Run `git status`. If you don't get any errors, that means all has gone well so far.
3. Run `git pull` this will make sure your local repo is the same as the remote
3. Setup your own branch to safely make changes to using this command: `git checkout -b <your_branch_name>`
    - NOTE: the name of the branch should be something logical related what you are changing
4. Make some changes
5. Stage + commit your changes to your branch in one command: `git commit -am "Message for my commit"`
6. To push your commits and your changes on your branch to remote (basically so the rest of the team can see it), 
   run this: `git push -u origin <your_branch_name>`
7. To update your local repo to sync any changes made to the main branch run this code snippet:
   ```
        git checkout master
        git pull
        git checkout <your_branch_name>
        git merge master
   ```
8. Ta da! You can now see the team's code changes and also share yours! 

## Get Familiar with Dev Environment

### IDE: VSCode

- Coding in the Terminal sucks, use the VSCode IDE to save lots of hours debugging and makes it easier to find bugs with autocomplete + suggestions
- Click on "Tools" in the top left corner, then click "IDE" to get to the IDE

### Rviz

To run the robot_arm simulation in Rviz, follow these steps:
1. Make sure you sourced your simulation_ws
2. Run `catkin_make` while still in the simulation_ws root directory to let ROS know of all the packages and launch files
3. Run `roslaunch arc_robot_arm_moveit_config demo.launch` to start the Rviz simulation
4. Click on "Tools" in the top left corner, then click "Graphical Tools"
5. Give it a second to load, but once it does, click Resize Window to get it to full screen and you should see the robot arm in Rviz!

Play around with more of the features! RDS/The Construct has tutorials on YouTube!
