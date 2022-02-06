/* Author: Raghava Uppuluri */

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "protoarm_kinematics/kinematics.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "test_vs");
	ros::NodeHandle node_handle("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	Kinematics robot_arm = Kinematics();

	geometry_msgs::Pose chessPiecePose;

	chessPiecePose.position.x = -0.17;
	chessPiecePose.position.y = 0;
	chessPiecePose.position.z = 0.15;

	robot_arm.moveToPoseGoal(chessPiecePose);

	return 0;
}
