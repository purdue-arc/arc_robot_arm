#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <ros.h>

#include <Servo.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>

#ifndef PROTOARM_H
#define PROTOARM_H

#define Rev1_PIN 3
#define Rev2_PIN 5
#define Rev3_PIN 6
#define Rev4_PIN 9
#define Rev5_PIN 10
#define GRIPPER_PIN 11

#define DOF 5 
#define OFFSET 90 // Offset angle from sim to real robot
#define UPPER_REAL_GRIP 180
#define LOWER_REAL_GRIP 90
#define UPPER_REAL_ARM 180
#define LOWER_REAL_ARM 0

#define UPPER_SIM_GRIP 45
#define LOWER_SIM_GRIP 0
#define UPPER_SIM_ARM 90
#define LOWER_SIM_ARM -90

namespace protoarm {

  // INTERNAL
  
  const int JOINT_CNT_ = 5;

	const int ARM_PINS_[JOINT_CNT_] = { 3, 5, 6, 9, 10 };     // order: base, shoulder, elbow, wrist-roll, wrist-pitch
  const float INIT_ARM_[JOINT_CNT_] = { 90,90,90,90,90 }; // initial angles in degrees [0,180]
  Servo arm_[JOINT_CNT_];

  const int GRIPPER_PIN_ = 11;
  const float INIT_GRIPPER_ = 180.0; // initial angle in degrees [90-180], 180 is open, 90 is tightly closed
  Servo gripper_;

  const void _ros_log_float(ros::NodeHandle nh, float f) { 
    char print_str[15]; // Leaves room for large numbers
    dtostrf(f, 9, 3, print_str); 
    nh.loginfo(print_str);
  }

  const float* _constrain_arm(float* arm) {
    return arm;
  }

  const float _constrain_gripper(float gripper) {
    return gripper;
  }

  // EXTERNAL 

  void init() {
    // attach servo pins and initialize angle 
    /*
    for(int i = 0; i < JOINT_CNT_; i++) {
      arm_[i].attach(ARM_PINS_[i]);
      arm_[i].write(INIT_ARM_[i]);
    }
    gripper_.attach(GRIPPER_PIN_);
    gripper_.write(INIT_GRIPPER_);
    */
  }

  void drive_arm(const sensor_msgs::JointState& joint_state, ros::NodeHandle nh) {
    float* arm = _constrain_arm(joint_state.position);
		nh.logerror("Arm:");  
    /*
    for(int i = 0; i < JOINT_CNT_; i++) {
		  _ros_log_float(nh, arm[i]);
    }
    */
  }

  void drive_gripper(const sensor_msgs::JointState& joint_state, ros::NodeHandle nh) {
    float gripper = _constrain_gripper(joint_state.position[0]);
		nh.logerror("gripper:");  
    /*
		_ros_log_float(nh, gripper);
    */
  }

  void log(ros::NodeHandle nh) {
		nh.logerror("Arm:");  
    for(int i = 0; i < JOINT_CNT_; i++) {
		  _ros_log_float(nh, arm_[i].read());
    }

    nh.logerror("Gripper:");  
		_ros_log_float(nh, gripper_.read());
  }
}

#endif
