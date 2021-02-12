/*
 * inverse_kinematics.h
 *
 *  Created on: 31-Aug-2016
 *      Author: Raghava 
 */

#ifndef INVERSE_KINEMATICS_5DOF
#define INVERSE_KINEMATICS_5DOF


#include <eigen3/Eigen/Dense>
#include <moveit/robot_state/robot_state.h>
#include <vector>

namespace mvp_arm_kinematics_plugin {
	class InverseKinematics5Dof {
		public:
			InverseKinematics5Dof(Eigen::Array<Eigen::Affine3d,Eigen::Dynamic,1>);
			bool JntToPose(void);
			bool PoseToJnt(void);
				
		private:
		Eigen::Array<Eigen::Affine3d,Eigen::Dynamic,1> joint_tfs;	

	};
};
#endif
