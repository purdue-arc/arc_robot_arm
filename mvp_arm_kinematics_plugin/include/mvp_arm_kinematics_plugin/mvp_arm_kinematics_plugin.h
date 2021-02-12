#ifndef MVP_ARM_KINEMATICS_PLUGIN_H
#define MVP_ARM_KINEMATICS_PLUGIN_H

// ROS
#include <ros/ros.h>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// MoveIt
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/kdl_kinematics_plugin/joint_mimic.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "inverse_kinematics_5dof.h"

namespace mvp_arm_kinematics_plugin {
	class MVPArmKinematicsPlugin : public kinematics::KinematicsBase {
		public:		
			MVPArmKinematicsPlugin();

			bool getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
				std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
				const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

			bool searchPositionIK(
				const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
				std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
				const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

			bool searchPositionIK(
				const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
				const std::vector<double>& consistency_limits, std::vector<double>& solution,
				moveit_msgs::MoveItErrorCodes& error_code,
				const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

			bool searchPositionIK(
				const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
				std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
				const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

			bool searchPositionIK(
				const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
				const std::vector<double>& consistency_limits, std::vector<double>& solution,
				const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
				const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

			bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
											 std::vector<geometry_msgs::Pose>& poses) const override;

			bool initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
										const std::string& base_frame, const std::vector<std::string>& tip_frames,
										double search_discretization) override;

			/**
			* @brief  Return all the joint names in the order they are used internally
			*/
			const std::vector<std::string>& getJointNames() const override;

			/**
			* @brief  Return all the link names in the order they are represented internally
			*/
			const std::vector<std::string>& getLinkNames() const override;
		private:
			moveit_msgs::KinematicSolverInfo _solver_info;	
			std::unique_ptr<mvp_arm_kinematics_plugin::InverseKinematics5Dof> _solver;
	};
};
#endif // MVP_ARM_KINEMATICS_PLUGIN
