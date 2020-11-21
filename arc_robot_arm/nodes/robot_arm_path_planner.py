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

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class RobotArmPathPlanner(object):
    "RobotArmPlanner"

    def __init__(self, arm_mg_name, gripper_mg_name, solver="MoveIt"):
        super(RobotArmPathPlanner, self).__init__()
        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('RobotArmPathPlanner', anonymous=True)

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        # Used for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        # arm_move_group to plan and execute motions:
        arm_move_group = moveit_commander.MoveGroupCommander(arm_mg_name)
        
        # default params that worked well
        arm_move_group.set_planning_time(10)
        arm_move_group.set_num_planning_attempts(5)
        
        # gripper arm_move_group:
        gripper_move_group = moveit_commander.MoveGroupCommander(gripper_mg_name)
        print('here')
        # Initialization
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.arm_move_group = arm_move_group
        self.gripper_move_group = gripper_move_group
        self.solver=solver
    
    ## Arm Path Planning

    def plan_to_pose_goal(self, pose_goal):
        if(callable(self.solver)):
            joints = self.solver(pose_goal)
            if(type(joints) is JointState):
               return self.arm_move_group.plan(joints)
            else:
                raise Exception("Custom solver does not return JointState type")
        else:
            return self.arm_move_group.plan(pose_goal)
    
    def go_to_pose_goal(self, pose_goal):
        plan = self.plan_to_pose_goal(pose_goal)
        self.execute_plan(plan)

    def plan_to_joint_goal(self, joint_goal):
        if(type(joint_goal) is JointState):
            return self.arm_move_group.plan(joint_goal)
        else:
            raise Exception("joint_goal is not JointState type")
    
    def go_to_joint_goal(self, joint_goal):
        plan = self.plan_to_joint_goal(joint_goal)
        self.execute_plan(plan)

    def execute_plan(self,plan): 
        self.arm_move_group.execute(plan,wait=True)
        
        # Calling `stop()` ensures that there is no residual movement
        self.arm_move_group.stop()
        
        self.arm_move_group.clear_pose_targets()
    
    def go_home(self):
        joints = JointState()
        joints.name = ['Rev1', 'Rev2', 'Rev3', 'Rev4']
        joints.position = [0,0,0,0]

        plan = self.go_to_joint_goal(joints)
    def move_gripper(self, theta):
        joint_arr = [theta, -1 * theta]
        joint_state = JointState()
        joint_state.position = joint_arr
        joint_state.name = self.gripper_move_group.get_joints()
        plan = self.gripper_move_group.plan(joint_state)

        self.gripper_move_group.execute(plan,wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.gripper_move_group.stop()

        self.gripper_move_group.clear_pose_targets()

