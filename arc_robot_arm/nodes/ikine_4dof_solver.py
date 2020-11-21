#!/usr/bin/env python3

# Try to print out the python version
import sys
import platform

import math
import numpy as np
import roboticstoolbox as rtb
from spatialmath import *
from spatialmath.base import q2r

class Ikine4DOFSolver:
    def __init__(self, dof):
        # CONSTANTS

        # DH Parameters
        links = np.array([.05801, 0.12015, 0.1330, 0.097766])  # Links of

        link1 = rtb.RevoluteDH(d=links[0], alpha=np.pi/2)
        link2 = rtb.RevoluteDH(a=links[1], alpha=np.pi)
        link3 = rtb.RevoluteDH(a=links[2], alpha=0)
        link4 = rtb.RevoluteDH(a=links[3], alpha=-np.pi/2)

        DH_Table = [link1, link2, link3, link4]

        # Transformation vector from IK simulator joint space to actual joint space
        self.transf_vec = np.array([-np.pi, -np.pi / 2, -np.pi / 2, 0])
        self.dof = dof
        self.eef_dof = 3
        
        self.mask = self.eef_dof * [1] + (6 - self.eef_dof) * [0] # Defines the dof of robot eef [x,y,z,roll,pitch,yaw]
        self.robot = rtb.DHRobot(DH_Table, name="4DOF MVP Robot Arm")

    def ikine(self, goal_pose):
        orient = Quaternion(goal_pose['orientation'])
        pos = list(goal_pose['position'])
        
        print(orient)

        goal_pose = SE3(pos[0],pos[1],pos[2]) * SE3(orient.matrix)
        joints, failure, reason = self.robot.ikine(
            goal_pose, mask=self.mask, ilimit=1000)

        if (failure):
            raise Exception(reason)

        joints = np.add(joints, self.transf_vec)

        
        return np.unwrap(joints)
