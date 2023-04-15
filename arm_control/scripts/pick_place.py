#! /usr/bin/env python3

import csv
import ctypes
import numpy as np
import os
import rospkg
import rospy
import sys
import transformations as T
import yaml

from timeit import default_timer as timer
from geometry_msgs.msg import Pose, Twist, Vector3
from relaxed_ik_ros1.msg import EEPoseGoals, EEVelGoals
from robot import Robot

path_to_src = rospkg.RosPack().get_path('relaxed_ik_ros1') + '/relaxed_ik_core'

above_board = [1.5865406156100033, 2.228143180472962, 0.0, -0.2198049105363947, 0.8423966125099708, 1.6885019773976913]
scan = []

class PickPlace:
    def __init__(self):
        try:
            tolerances = rospy.get_param('~tolerances')
        except:
            print("No tolerances are given, using zero tolerances")
            tolerances = [0, 0, 0, 0, 0, 0]

        self.tolerances = []

        assert len(tolerances) % 6 == 0, "The number of tolerances should be a multiple of 6"
        for i in range(int(len(tolerances) / 6)):
            self.tolerances.append(Twist(   Vector3(tolerances[i*6],    tolerances[i*6+1], tolerances[i*6+2]),
                                            Vector3(tolerances[i*6+3],  tolerances[i*6+4], tolerances[i*6+5])))

        deault_setting_file_path = path_to_src + '/configs/settings.yaml'

        setting_file_path = rospy.get_param('setting_file_path')

        if setting_file_path == '':
            setting_file_path = deault_setting_file_path

        os.chdir(path_to_src )

        # Load the infomation
        setting_file = open(setting_file_path, 'r')
        settings = yaml.load(setting_file, Loader=yaml.FullLoader)

        urdf_file = open(path_to_src + '/configs/urdfs/' + settings["urdf"], 'r')
        urdf_string = urdf_file.read()
        rospy.set_param('robot_description', urdf_string)

        self.robot = Robot(setting_file_path)
        self.starting_ee_poses =  self.robot.fk(settings['starting_config'])
        print(self.starting_ee_poses)

        self.trajectory = self.generate_trajectory()
        self.ee_pose_pub = rospy.Publisher('relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)

        count_down_rate = rospy.Rate(1)
        count_down = 3
        while not rospy.is_shutdown():
            print("Start line tracing in {} seconds".format(count_down))
            count_down -= 1
            if count_down == 0:
                break
            count_down_rate.sleep()

        self.trajectory_index = 0
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def generate_trajectory(self):
        num_points = 50
        trajectory = []
        x = []
        y = []
        z = []
        for k in range(self.robot.num_chain):
            z.append(self.starting_ee_poses[k].position.z)
            y.append(self.starting_ee_poses[k].position.y)

        delta = 0.3 / num_points
        dz = [0, -delta, 0, delta]
        dy = [delta, 0, -delta, 0]

        for i in range(4):
            for j in range(num_points):
                poses = self.copy_poses(self.starting_ee_poses)
                for k in range(self.robot.num_chain):
                    poses[k].position.z = z[k]
                    poses[k].position.y = y[k]
                    z[k] += dz[i]
                    y[k] += dy[i]
                trajectory.append(poses)

        return trajectory

    def copy_poses(self, input_poses):
        output_poses = []
        for i in range(len(input_poses)):
            output_poses.append(self.copy_pose(input_poses[i]))
        return output_poses

    def copy_pose(self, input_pose):
        output_pose = Pose()
        output_pose.position.x = input_pose.position.x
        output_pose.position.y = input_pose.position.y
        output_pose.position.z = input_pose.position.z
        output_pose.orientation.x = input_pose.orientation.x
        output_pose.orientation.y = input_pose.orientation.y
        output_pose.orientation.z = input_pose.orientation.z
        output_pose.orientation.w = input_pose.orientation.w
        return output_pose

    def timer_callback(self, event):
        if self.trajectory_index >= len(self.trajectory):
            self.timer.shutdown()
            return

        ee_pose_goals = EEPoseGoals()
        for i in range(self.robot.num_chain):
            ee_pose_goals.ee_poses.append(self.trajectory[self.trajectory_index][i])
            if i < len(self.tolerances):
                ee_pose_goals.tolerances.append(self.tolerances[i])
            else:
                ee_pose_goals.tolerances.append(self.tolerances[0])
        self.ee_pose_pub.publish(ee_pose_goals)
        self.trajectory_index += 1

if __name__ == '__main__':
    rospy.init_node('PickPlace')
    pick_place = PickPlace()
    rospy.spin()
