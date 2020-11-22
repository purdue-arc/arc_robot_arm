#!/usr/bin/env python

"""
Node to send path planning commands to Robot arm MoveGroup.
"""

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import rospy
import moveit_commander
import sys
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from robot_arm_path_planner import RobotArmPathPlanner
from client_ik import ikine4dof
import time

# Box dims
box_width = 0.059
box_length = 0.048
box_height = 0.054

# offsets
pick_offset = 0.05
release_offset = 0.02

# Retreat
retreat_angle = pi/12
# box locations

def get_gripper_joint_from_distance(dist) :
    # linear regression to go from distance between gripper fingers
    # to joint values
    m = 9.0623
    b = -0.0050

    return m * dist + b

def retreat(planner):
    joints = JointState()
    joints.position = planner.arm_move_group.get_current_joint_values()
    joints.position[1] = joints.position[1] + retreat_angle # moves gripper out of the way 
    joints.name = planner.arm_joint_names
    planner.go_to_joint_goal(joints)
    planner.go_home()

def pick_box(planner):
    dist = box_width - pick_offset
    joint = get_gripper_joint_from_distance(dist)
    planner.move_gripper(joint)

    retreat(planner)

def go_to_high_pose(planner):
    joints = [0,0,-pi/6,pi/6]
    joint_state = JointState()
    joint_state.position = joints
    joint_state.name = planner.arm_joint_names
    planner.go_to_joint_goal(joint_state)

def release_box(planner):
    planner.move_gripper(45) ## release

    retreat(planner)

if __name__=='__main__':
    planner = RobotArmPathPlanner('arm','gripper',ikine4dof)
    
    z_offset = 0.11 # to account for base in ik
    
    box1_start = Pose()
    box1_start.position.x = -0.16
    box1_start.position.y = 0.205
    box1_start.position.z = box_height * 2 - z_offset # 2 box height
    box1_start.orientation.w = 1

    box1_end = Pose()
    box1_end.position.x = box1_start.position.x
    box1_end.position.y = box1_start.position.y * -1
    box1_end.position.z = box1_start.position.z - box_height
    box1_end.orientation.w = 1
    
    box2_start = Pose()
    box2_start.position.x = box1_start.position.x
    box2_start.position.y = box1_start.position.y
    box2_start.position.z = box_height - z_offset
    box2_start.orientation.w = 1

    box2_end = Pose()
    box2_end.position.x = box2_start.position.x
    box2_end.position.y = box2_start.position.y * -1
    box2_end.position.z = box1_start.position.z
    box2_end.orientation.w = 1
    
    planner.go_to_pose_goal(box1_start)
    pick_box(planner)
    
    planner.go_to_pose_goal(box1_end)
    release_box(planner)
    
    planner.go_to_pose_goal(box2_start)
    pick_box(planner)

    planner.go_to_pose_goal(box2_end)
    release_box(planner)
    ''' 
    go_to_high_pose(planner)

    # Invert motions, put boxes back
    planner.go_to_pose_goal(box2_end)
    pick_box(planner)

    planner.go_to_pose_goal(box2_start)
    release_box(planner)

    planner.go_to_pose_goal(box1_end)
    pick_box(planner)

    planner.go_to_pose_goal(box1_start)
    release_box(planner)
    '''
