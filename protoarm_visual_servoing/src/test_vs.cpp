/* Author: Raghava Uppuluri */

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "protoarm_kinematics/kinematics.h"

std::vector<geometry_msgs::Pose> testFileToPoses(std::string test_file_name) {
	std::vector<geometry_msgs::Pose> test_goal_poses;
	std::ifstream infile(test_file_name);	
	std::string line;

	while (std::getline(infile, line)) {
		if(line.at(0) != '#') {  // NOTE: Comment lines in test txt files start with '#'
			std::istringstream iss(line);
			double x, y, z;
			if (!(iss >> x >> y >> z)) { break; } // error

			ROS_INFO_STREAM(x << " " << y << " " << z << "\n");

			geometry_msgs::Pose pose;
			pose.position.x = x;
			pose.position.y = y;
			pose.position.z = z;
			test_goal_poses.push_back(pose);	
		}
}

return test_goal_poses;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "test_vs");
	ros::NodeHandle node_handle("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ROS_INFO("Waiting for context to come up");

	Kinematics robot_arm = Kinematics();

	return 0;
}
