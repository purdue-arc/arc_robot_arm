/*
 * Test single dynamixel servo
 *
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <DynamixelShield.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>

using namespace ControlTableItem;

const uint8_t J6 = 6;

const float DXL_PROTOCOL_VERSION = 1.0;

DynamixelShield dxl;

float servo;
ros::NodeHandle nh;

char print_str[15]; // used to print out 15 char outputs

inline float rad_to_deg(float position_radians)
{
  return position_radians * 57.2958;
}

/* Sets arm joint values from JointState msg *  and maps them to real arm space
*/
void set_servo(const std_msgs::Float64& value) {
  servo = rad_to_deg(value.data);
}

void print_servo(){
    dtostrf(servo, 9, 3, print_str); // Leave room for too large numbers!
    nh.loginfo(print_str);
}

void drive_servo(){
    nh.loginfo("Driving:");
		dxl.setGoalPosition(J6, servo, UNIT_DEGREE);
}

/* Callback for JointState subscriber, runs when new JointState
*  is published to move_group/fake_controller_joint_states
*/
void joint_control_cb(const std_msgs::Float64& value) { // output can be either for gripper joint or for arm joints
    set_servo(value);
    nh.loginfo("arm");
    nh.loginfo("Joint Value:");
    print_servo();
    drive_servo();
}

ros::Subscriber<std_msgs::Float64> sub("test_servo", joint_control_cb);

void setup(){
	dxl.begin(1000000);
	dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
	dxl.ping(J6);

	dxl.torqueOff(J6);
	dxl.setOperatingMode(J6, OP_POSITION);
	dxl.torqueOn(J6);

  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  nh.spinOnce();
  delay(5);
}
