/*
 * inverse_kinematics.cpp
 *
 *  Created on: 20-Jan-2021
 *      Author: Raghava 
 */
#include <mvp_arm_kinematics_plugin/inverse_kinematics_5dof.h>
#include <eigen3/Eigen/Dense>
#include <iostream> 
//#include <ginac/ginac.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>

using namespace std;
namespace mvp_arm_kinematics_plugin {
	InverseKinematics5Dof::InverseKinematics5Dof(Eigen::Array<Eigen::Affine3d,Eigen::Dynamic,1> _joint_tfs) {
		joint_tfs = _joint_tfs;	
	}

	bool InverseKinematics5Dof::JntToPose(){
		return true;	
	}

	bool InverseKinematics5Dof::PoseToJnt(){
		return true;	
	}
};
