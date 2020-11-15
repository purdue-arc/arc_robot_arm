/*
 * rosserial interface with 4dof robot arm
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>

#define Rev1_PIN 11
#define Rev2_PIN 10
#define Rev3_PIN 9
#define Rev4_PIN 6
#define Rev5_PIN 5
#define Rev6_PIN 4
#define DOF 5

ros::NodeHandle  nh;

Servo rev1;
Servo rev2;
Servo rev3;
Servo rev4;
Servo rev5;
Servo rev6;

char print_str[15];

float gripper;
float arm[DOF];

inline float RadiansToDegrees(float position_radians)
{
  return position_radians * 57.2958;
}

void set_gripper_joints(const sensor_msgs::JointState& joint_state){
  gripper = RadiansToDegrees(joint_state.position[1]); // left finger actuation only
}

void set_arm_joints(const sensor_msgs::JointState& joint_state) {
  arm[0] = RadiansToDegrees(joint_state.position[0]);
  arm[1] = RadiansToDegrees(joint_state.position[1]);
  arm[2] = RadiansToDegrees(joint_state.position[2]);
  //arm[3] = joint_state.position[3];
  arm[4] = RadiansToDegrees(joint_state.position[3]);
}

int is_arm_move_group(const sensor_msgs::JointState& joint_state){
  String first_joint = joint_state.name[0]; 
  String arm_first_joint = "Rev1";
  nh.loginfo(joint_state.name[0]);
  return first_joint == arm_first_joint;
}

void print_arm(){
  for(int i = 0; i < DOF; i++) {
    dtostrf(arm[i], 9, 3, print_str); // Leave room for too large numbers!
    nh.loginfo(print_str);
  }
}

void print_gripper(){
  dtostrf(gripper, 9, 3, print_str);
  nh.loginfo(print_str);
}

void joint_control_cb( const sensor_msgs::JointState& joint_state) { // output can be either for gripper joint or for arm joints
  if(is_arm_move_group(joint_state))  {
    set_arm_joints(joint_state);
    nh.loginfo("arm");
    print_arm();
  }
  else{
    set_gripper_joints(joint_state);
    nh.loginfo("gripper");
    print_gripper();
  }
}

ros::Subscriber<sensor_msgs::JointState> sub("move_group/fake_controller_joint_states", joint_control_cb);

void setup(){
  Serial.begin(4800);
  nh.initNode();
  nh.subscribe(sub);
  
  rev1.attach(Rev1_PIN);
  rev2.attach(Rev2_PIN);
  rev3.attach(Rev3_PIN);
  rev4.attach(Rev4_PIN);
  rev5.attach(Rev5_PIN);
  rev6.attach(Rev6_PIN);
  rev1.write(90);
  rev2.write(90);
  rev3.write(90);
  rev4.write(90);
  rev5.write(90);
  rev6.write(90);

}

void loop(){
  //nh.spinOnce();
  delay(1);
}
