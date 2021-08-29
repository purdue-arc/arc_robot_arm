#include "protoarm.h"

namespace protoarm {
  void init() {
    // attach servo pins and initialize angle 
    for(int i = 0; i < JOINT_CNT_; i++) {
      arm[i].attach(ARM_PINS_[i]);
      arm[i].write(INIT_ARM_[i]);
    }
    gripper.attach(GRIPPER_PIN_);
    gripper.write(INIT_GRIPPER_);
  }

  float* constrain_arm(float* arm) {
    return arm;
  }

  float constrain_gripper(float gripper) {
    return gripper;
  }

  void drive_arm(const sensor_msgs::JointState& joint_state, ros::NodeHandle nh) {
    float arm[JOINT_CNT_] = constrain_arm(joint_state.position);
		nh.logerror("Arm:");  
    for(int i = 0; i < JOINT_CNT; i++) {
		  nh.logerror(String(joint_state.position[i]).c_str());
    }
  }

  void drive_gripper(const sensor_msgs::JointState& joint_state, ros::NodeHandle nh) {
    float gripper = joint_state.position[0];
    nh.logerror("Gripper:");  
		nh.logerror(String(joint_state.position[0]).c_str());
  }

  void log(ros::NodeHandle nh) {
		nh.logerror("Arm:");  
    for(int i = 0; i < JOINT_CNT; i++) {
		  nh.logerror(String(arm[i].read()).c_str());
    }
    nh.logerror("Gripper:");  
    nh.logerror(String(gripper.read()).c_str());
  }
} 
