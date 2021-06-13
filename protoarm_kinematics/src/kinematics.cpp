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

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/PlanningScene.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <protoarm_kinematics/kinematics.h>

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
	
  ROS_INFO("Trying pose goal move.....");
  move_group_->setApproximateJointValueTarget(goal_pose);

	bool plan_sucesss = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("kinematics-info", "Moving to pose goal %s", plan_success ? "SUCCESS" : "FAILURE");

  robot_trajectory::RobotTrajectory rt(move_group_.getCurrentState()->getRobotModel(), "arm");
  rt.setRobotTrajectoryMsg(*move_group_.getCurrentState(), trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool success = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s",success?"SUCCEEDED":"FAILED");
  rt.getRobotTrajectoryMsg(trajectory);

  plan.trajectory_ = trajectory;
  ROS_INFO("Visualizing plan (%.2f%% acheived)",fraction * 100.0);

  if (success) {
    move_group_.execute(plan);
    return true;
  } 
  else {
    ROS_WARN("no success");
    return false;
  }
} 

bool Kinematics::moveToJointGoal(sensor_msgs::JointState joint_state) {
  ROS_INFO("Trying joint goal move.....");
  move_group_->setJointValueTarget(joint_state);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
	bool plan_success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  plan.trajectory

  ROS_INFO_NAMED("kinematics-info", "Moving to joint goal %s", plan_success ? "SUCCESS" : "FAILURE");
	
	return plan_success;
}
 
bool Kinematics::isClose(geometry_msgs::Pose goal, double tolerance) {
	const geometry_msgs::Pose actual = move_group_->getCurrentPose().pose;

	double x_err = std::abs(actual.position.x - goal.position.x); 	
	double y_err = std::abs(actual.position.y - goal.position.y); 	
	double z_err = std::abs(actual.position.z - goal.position.z); 	

	return sqrt(pow(x_err, 2) + pow(y_err, 2) + pow(z_err, 2)) < tolerance;
}	
