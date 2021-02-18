/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *********************************************************************/
/* Author: Raghava Uppuluri */

#include <math.h>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/PlanningScene.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <arc_robot_arm/kinematics.h>

Kinematics::Kinematics() { 
	move_group_ = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
	
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  robot_model_ = robot_model_loader.getModel();

  robot_state_.reset(new robot_state::RobotState(robot_model_));

	// Set Params	
	move_group_->setMaxVelocityScalingFactor(VELOCITY_SCALING_FACTOR);
	move_group_->setPlanningTime(PLANNING_TIME);

}

void Kinematics::printRobotState() {
  const robot_state::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(PLANNING_GROUP);

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  std::vector<double> joint_values;
  std::cout << "EEF Name: " << joint_model_group->getEndEffectorName() << std::endl;
  ROS_INFO_NAMED("kinematics-info", "Reference frame: %s", move_group_->getPlanningFrame().c_str());

  ROS_INFO_NAMED("kinematics-info", "End effector link: %s", move_group_->getEndEffectorLink().c_str());
} 

bool Kinematics::moveToPoseGoal(geometry_msgs::Pose goal_pose, double tolerance = -1.0) {
	
  ROS_INFO("Trying IK.....");
  move_group_->setApproximateJointValueTarget(goal_pose);

	bool moveItSuccess = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("kinematics-info", "Moving to pose goal %s", moveItSuccess ? "SUCCESS" : "FAILURE");

	if(tolerance != -1.0) {
		bool withinTolerance = Kinematics::isClose(goal_pose, tolerance);
		ROS_INFO_NAMED("kinematics-info", "Within Tolerance %s", withinTolerance ? "SUCCESS" : "FAILURE");
	}

	return moveItSuccess;
} 

// TODO: Yet to be implemented
bool Kinematics::moveToJointGoal(sensor_msgs::JointState) {
	return true;
}
 
bool Kinematics::isClose(geometry_msgs::Pose goal, double tolerance) {
	const geometry_msgs::Pose actual = move_group_->getCurrentPose().pose;

	double x_err = std::abs(actual.position.x - goal.position.x); 	
	double y_err = std::abs(actual.position.y - goal.position.y); 	
	double z_err = std::abs(actual.position.z - goal.position.z); 	

	return sqrt(pow(x_err, 2) + pow(y_err, 2) + pow(z_err, 2)) < tolerance;
}	
