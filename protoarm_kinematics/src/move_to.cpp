/* Author: Raghava Uppuluri */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <protoarm_kinematics/kinematics.h>

static const std::string MOVE_TO_POSE_SUB = "move_to_pose";

static const std::string MOVE_TO_JOINT_STATE_SUB = "move_to_joint_state";

class MoveGoalListener {
	public:
		Kinematics robot_arm;

		void move_to_pose_cb(const geometry_msgs::PoseStamped&);
		void move_to_joint_state_cb(const sensor_msgs::JointState&);
};

void MoveGoalListener::move_to_pose_cb(const geometry_msgs::PoseStamped& goal_pose) {
	
  ROS_INFO("Message Recieved");

	robot_arm.moveToPoseGoal(goal_pose.pose);	
}

void MoveGoalListener::move_to_joint_state_cb(const sensor_msgs::JointState& goal_state) {
	
  ROS_INFO("Message Recieved");

	robot_arm.moveToJointGoal(goal_state);	
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_to_pose");
  ros::NodeHandle nh;
	
	// Async spinner used as Kinematics object calls getCurrentPose which needs the spinner 
	// in a second thread
	ros::AsyncSpinner spinner(2);
	spinner.start();

  ROS_INFO("Waiting for MoveIt context to come up");

	MoveGoalListener listener;

	ros::Subscriber move_to_pose_sub = nh.subscribe(MOVE_TO_POSE_SUB, 1000, &MoveGoalListener::move_to_pose_cb, &listener);

	ros::Subscriber move_to_joint_state_sub = nh.subscribe(MOVE_TO_JOINT_STATE_SUB, 1000, &MoveGoalListener::move_to_joint_state_cb, &listener);

	ros::waitForShutdown();

	return 0;
}
