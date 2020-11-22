/*
 * Rosserial interface with 4dof robot arm
 *
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

#define Rev1_PIN 3
#define Rev2_PIN 5
#define Rev3_PIN 6
#define Rev4_PIN 9
#define Rev5_PIN 10
#define Rev6_PIN 11

#define DOF 5 // Only 4 are used
#define OFFSET 90 // Offset angle from sim to real robot

ros::NodeHandle  nh;

Servo rev1; // base servo
Servo rev2; // shoulder servo
Servo rev3; // elbow servo
Servo rev4; // wrist roll servo
Servo rev5; // wrist pitch servo
Servo rev6; // gripper servo

char print_str[15]; // used to print out 15 char outputs

float gripper = 45.0; // angle in degrees [0-45]
float arm[DOF] = {90,90,90,90,90}; // angles in degrees [0,180]

inline float RadiansToDegrees(float position_radians)
{
  return position_radians * 57.2958;
}

int val_arm_joints() {
  for(int i = 0; i < DOF; i++){
    if(arm[i] < 0 || arm[i] > 180){
      return 0;
    }
  }
  return 1;
}

int val_gripper_joint() {
  return gripper >= 0 && gripper <= 45;
}

// Sets gripper joint from JointState msg
void set_gripper_joint(const sensor_msgs::JointState& joint_state){
  gripper = abs(RadiansToDegrees(joint_state.position[1])); // left finger actuation only
}

/* Sets arm joint values from JointState msg 
*  and maps them to real arm space
*/
void set_arm_joints(const sensor_msgs::JointState& joint_state) {
  arm[0] = RadiansToDegrees(joint_state.position[0]) + OFFSET;
  arm[1] = RadiansToDegrees(joint_state.position[1]) + OFFSET;
  arm[2] = RadiansToDegrees(joint_state.position[2]) + OFFSET;
  //arm[3] = joint_state.position[3] + OFFSET;
  arm[4] = RadiansToDegrees(joint_state.position[3]) * -1 + OFFSET;
}

// checks whether the JointState msg is for the arm or gripper movegroup
int is_arm_move_group(const sensor_msgs::JointState& joint_state){
  // Name of first joint (either Rev1 or Rev5)
  String first_joint = joint_state.name[0]; 
  String arm_first_joint = "Rev1"; // first joint of arm

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

void drive_arm(){
  if(val_arm_joints()){
    rev1.write(arm[0]);
    rev2.write(arm[1]);
    rev3.write(arm[2]);
    //rev4.write(arm[3]);
    rev5.write(arm[4]);
  }
  else{
    nh.loginfo("Invalid Arm Joint Value!");
  }
}

void drive_gripper(){
  if(val_gripper_joint()){
    rev6.write(gripper);
  }
  else{
    nh.loginfo("Invalid Gripper Joint Value!");
  }
}

/* Callback for JointState subscriber, runs when new JointState
*  is published to move_group/fake_controller_joint_states
*/
void joint_control_cb(const sensor_msgs::JointState& joint_state) { // output can be either for gripper joint or for arm joints
  if(is_arm_move_group(joint_state))  {
    set_arm_joints(joint_state);
    nh.loginfo("arm");
    print_arm();
    drive_arm();    
  }
  else{
    set_gripper_joint(joint_state);
    nh.loginfo("gripper");
    print_gripper();
    drive_gripper();
  }
}

ros::Subscriber<sensor_msgs::JointState> joints_sub("move_group/fake_controller_joint_states", joint_control_cb);

void setup(){
  Serial.begin(4800);
  nh.initNode();
  nh.subscribe(joints_sub);
  
  rev1.attach(Rev1_PIN);
  rev2.attach(Rev2_PIN);
  rev3.attach(Rev3_PIN);
  rev4.attach(Rev4_PIN);
  rev5.attach(Rev5_PIN);
  rev6.attach(Rev6_PIN);
  
  rev1.write(arm[0]);
  rev2.write(arm[1]);
  rev3.write(arm[2]);
  rev4.write(arm[3]);
  rev5.write(arm[4]);
  rev6.write(gripper);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
