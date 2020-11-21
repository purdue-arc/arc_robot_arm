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
from client_ik import request_ik_solution
import time

# Box dims
box_width = 0.059
box_length = 0.048
box_height = 0.054

def get_gripper_joint_from_distance(dist) :
    # linear regression to go from distance between gripper fingers
    # to joint values
    m = 9.0623
    b = -0.0050

    return m * dist + b

def pick_box(planner):
    offset = 0.05 # to account for innaccuracies
    dist = box_width - offset
    joint = get_gripper_joint_from_distance(dist)
    planner.move_gripper(joint)

def release_box(planner):
    offset = 0.02
    dist = box_width + offset
    joint = get_gripper_joint_from_distance(dist)
    planner.move_gripper(joint)
    joints = JointState()
    joints.position = planner.arm_move_group.get_current_joint_values()
    joints.position[1] = joints.position[1] + pi/12 # moves gripper out of the way 
    joints.name = planner.arm_move_group.get_joints()
    planner.go_to_joint_goal(joints)


if __name__=='__main__':
    planner = RobotArmPathPlanner('arm','gripper',request_ik_solution)
    
    # Box dims
    box_width = 0.059
    box_length = 0.048
    box_height = 0.054
    
    z_offset = 0.11
    
    box1_start = Pose()
    box1_start.position.x = -0.16
    box1_start.position.y = 0.205
    box1_start.position.z = box_height * 2 - z_offset
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

    box1_start_joints = request_ik_solution(planner.arm_move_group.get_joints(), box1_start)
    
    box1_end_joints = request_ik_solution(planner.arm_move_group.get_joints(), box1_end)

    box2_start_joints = request_ik_solution(planner.arm_move_group.get_joints(), box2_start)

    box2_end_joints = request_ik_solution(planner.arm_move_group.get_joints(), box2_end)

    planner.go_to_joint_goal(box1_start_joints)
    pick_box(planner)
    planner.go_to_joint_goal(box1_end_joints)
    release_box(planner)
    planner.go_home()
    planner.go_to_joint_goal(box2_start_joints)
    pick_box(planner)
    planner.go_home()
    planner.go_to_joint_goal(box2_end_joints)
    release_box(planner)
    planner.go_home()
