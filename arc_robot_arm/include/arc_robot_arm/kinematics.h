/* Author: Raghava Uppuluri */

#ifndef KINEMATICS
#define KINEMATICS

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/PlanningScene.h>

class Kinematics {
	public:
		Kinematics();  // Initializes MoveGroup API within ROS
		
		void printRobotState(); 
		bool moveToPoseGoal(geometry_msgs::Pose, double); 
		bool moveToJointGoal(sensor_msgs::JointState); 
		void setVelocityScalingFactor(double);

		bool isClose(geometry_msgs::Pose, double);  // Determines if eef is close to target		
	
		const std::string PLANNING_GROUP = "arm";
		const double VELOCITY_SCALING_FACTOR = 0.5;
		const double PLANNING_TIME = 2;

  	moveit::planning_interface::MoveGroupInterface* move_group_;
		robot_model::RobotModelPtr robot_model_;
		robot_state::RobotStatePtr robot_state_;
  	moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_;
};

#endif
