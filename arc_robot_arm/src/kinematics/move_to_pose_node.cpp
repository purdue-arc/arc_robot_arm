/* Author: Raghava Uppuluri */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <arc_robot_arm/kinematics.h>

static const std::string MOVE_TO_POSE_SUB = "move_to_pose";

class MoveGoalListener {
	public:
		Kinematics robot_arm;

		void move_to_pose_cb(const geometry_msgs::Pose&);
};

void MoveGoalListener::move_to_pose_cb(const geometry_msgs::Pose& goal_pose) {
	
  ROS_INFO("Message Recieved");

	double tolerance = 0.001; // meters
	robot_arm.moveToPoseGoal(goal_pose, tolerance);	
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_to_pose");
  ros::NodeHandle nh;

  ROS_INFO("Waiting for MoveIt context to come up");

	MoveGoalListener listener;

	ros::Subscriber move_to_pose_sub = nh.subscribe(MOVE_TO_POSE_SUB, 1000, &MoveGoalListener::move_to_pose_cb, &listener);

	ros::spin();
}
