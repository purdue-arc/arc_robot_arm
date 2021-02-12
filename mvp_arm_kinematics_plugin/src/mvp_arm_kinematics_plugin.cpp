// General
#include <vector>

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

#include <mvp_arm_kinematics_plugin/mvp_arm_kinematics_plugin.h>
#include <mvp_arm_kinematics_plugin/inverse_kinematics_5dof.h>
#include <pluginlib/class_list_macros.h>

namespace mvp_arm_kinematics_plugin {
	MVPArmKinematicsPlugin::MVPArmKinematicsPlugin() {
	}	

		bool MVPArmKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
      std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options ) const {
			return true;
		}

  	bool MVPArmKinematicsPlugin::searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options) const {
			
			return true;	
		} 

  	bool MVPArmKinematicsPlugin::searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options ) const {
			
			return true;	
		}
  	bool MVPArmKinematicsPlugin::searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options ) const {
		
			return true;
		} 

  	bool MVPArmKinematicsPlugin::searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options ) const {
			return true;			
		} 

  	bool MVPArmKinematicsPlugin::getPositionFK(
				const std::vector<std::string>& link_names, const std::vector<double>& joint_angles, 
				std::vector<geometry_msgs::Pose>& poses) const {
			return true;	
		} 

  	bool MVPArmKinematicsPlugin::initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                  const std::string& base_frame, const std::vector<std::string>& tip_frames,
                  double search_discretization) {
			storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
			std::vector<const moveit::core::JointModel* > joint_models = robot_model.getJointModels();	
			Eigen::Array<Eigen::Affine3d, Eigen::Dynamic, 1> robot_tfs;

			for(int i = 0; i < joint_models.size(); i++) {
				const moveit::core::LinkModel* link_model = joint_models[i]->getChildLinkModel();		
				robot_tfs << link_model->getJointOriginTransform();
			}
				
			_solver.reset(new mvp_arm_kinematics_plugin::InverseKinematics5Dof(robot_tfs));	
			return true;	
		} 

  	/**
   	* @brief  Return all the joint names in the order they are used internally
   	*/
  	const std::vector<std::string>& MVPArmKinematicsPlugin::getJointNames() const {
			return _solver_info.joint_names;	
		} 

  	/**
   	* @brief  Return all the link names in the order they are represented internally
   	*/
  	const std::vector<std::string>& MVPArmKinematicsPlugin::getLinkNames() const {
			return _solver_info.link_names;	
		} 
};

PLUGINLIB_EXPORT_CLASS(mvp_arm_kinematics_plugin::MVPArmKinematicsPlugin, kinematics::KinematicsBase)
