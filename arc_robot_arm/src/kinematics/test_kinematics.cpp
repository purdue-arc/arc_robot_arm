/* Author: Raghava Uppuluri */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

static const std::string TEST_FILE_NAME = "test_kinematics.txt"; // Must be in local path when running ROS node!!

std::vector<geometry_msgs::Pose> testFileToPoses() {
	std::vector<geometry_msgs::Pose> test_goal_poses;
	std::ifstream infile(TEST_FILE_NAME);	
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
  ros::init(argc, argv, "test_kinematics");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

	ros::Publisher test_goal_pose_pub = nh.advertise<geometry_msgs::Pose>("move_to_pose", 1000);

	ros::Rate poll_rate(100);

	// waits until subscribers are setup
	while(test_goal_pose_pub.getNumSubscribers() == 0) {
		poll_rate.sleep();
	}

	std::vector<geometry_msgs::Pose> test_goal_poses = testFileToPoses();

	geometry_msgs::Pose pose;
	pose.position.x = -0.2;
	pose.position.y = 0;
	pose.position.z = 0.15;

	for(int i = 0; i < test_goal_poses.size(); i++) {
		test_goal_pose_pub.publish(test_goal_poses[i]);	
	}

	return 0;
}
