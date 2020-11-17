#!usr/bin/env python3
import numpy as np
import roboticstoolbox as rtb
from pyquaternion import Quaternion
from spatialmath import *

class MVPArmIKSolver:
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

    def ikine(self, T, dof):

        joints_from_ik, failure, reason = self.robot.ikine(
            T, mask=self.mask, ilimit=1000)

        if (failure):
            raise Exception(reason)

        transf_joints = np.add(joints_from_ik, self.transf_vec)

        for i in range(0,dof):
            transf_joints[i] = (transf_joints[i] + 2 * np.pi) % (2 * np.pi)
        
        final_joints = list(map(np.degrees, transf_joints))
        return final_joints


def main():
    solver = MVPArmIKSolver(dof=4)

    T1 = SE3(-0.15, 0.05, 0.05) * SE3.RPY([0, 0, 0], order='xyz')

    T2 = SE3(-0.145075699259, -0.121377287933, 0.154914489919) * \
        SE3.RPY([-2.0226025, 0.8022815, -1.9848712], order='xyz')

    print(solver.ikine(T2, 4))


main()
