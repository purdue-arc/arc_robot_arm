#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <ros.h>

#include <Servo.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>

namespace protoarm {
  const int JOINT_CNT = 5;

  // ARM
	const int servo_pins[JOINT_CNT] = { 3, 5, 6, 9, 10 };     //order: base, shoulder, elbow, wrist-roll, wrist-pitch
  float arm[JOINT_CNT] = {90,90,90,90,90}; // angles in degrees [0,180]
  Servo servos[JOINT_CNT];

  // GRIPPER 

  const int gripper_pin = 11;
  float gripper = 180.0; // angle in degrees [90-180], 180 is open, 90 is tightly closed
  Servo gripper;

  void init();

	void drive_arm(const sensor_msgs::Actuation& actuation, ros::NodeHandle nh);
  void drive_gripper(const sensor_msgs::Actuation& actuation, ros::NodeHandle nh);
}
