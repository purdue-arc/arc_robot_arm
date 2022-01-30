/*
 * Rosserial driver for the protoarm
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

ros::NodeHandle  nh;

Servo rev1; // base servo
Servo rev2; // shoulder servo
Servo rev3; // elbow servo
Servo rev4; // wrist roll servo
Servo rev5; // wrist pitch servo
Servo rev6; // gripper servo

char print_str[15]; // used to print out 15 char outputs

float gripper = 165.0; // angle in degrees [90-180]
float arm[DOF] = {90,90,90,90,90}; // angles in degrees [0,180]

inline float RadiansToDegrees(float position_radians)
{
  return position_radians * 57.2958;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int val_arm_joints() {
  for(int i = 0; i < DOF; i++){
    if(arm[i] < LOWER_REAL_ARM || arm[i] > UPPER_REAL_ARM){
      return 0;
    }
  }
  return 1;
}

float constrain_arm_joint(float pos) {
  return mapf(RadiansToDegrees(pos), LOWER_SIM_ARM, UPPER_SIM_ARM, LOWER_REAL_ARM, UPPER_REAL_ARM);
}
float constrain_gripper_joint(float pos) {
  return mapf(constrain(RadiansToDegrees(pos),5,45), LOWER_SIM_GRIP, UPPER_SIM_GRIP, LOWER_REAL_GRIP, UPPER_REAL_GRIP); // left finger actuation only
}

int val_gripper_joint() {
  return gripper >= LOWER_REAL_GRIP && gripper <= UPPER_REAL_GRIP;
}

// Sets gripper joint from JointState msg
void set_gripper_joint(const sensor_msgs::JointState& joint_state){
  gripper = constrain_gripper_joint(joint_state.position[0]);
}

/* Sets arm joint values from JointState msg 
*  and maps them to real arm space
*/
void set_arm_joints(const sensor_msgs::JointState& joint_state) {
  arm[0] = constrain_arm_joint(joint_state.position[0]);
  arm[1] = constrain_arm_joint(joint_state.position[1]);
  arm[2] = constrain_arm_joint(joint_state.position[2]);
  //arm[3] = constrain_arm_joint(joint_state.position[3]);
  arm[4] = constrain_arm_joint(-1 * joint_state.position[3]);
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
    rev4.write(arm[3]);
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

ros::Subscriber<sensor_msgs::JointState> joints_sub("/move_group/fake_controller_joint_states", joint_control_cb);

void setup(){
  Serial.begin(4800);
  nh.initNode();
  nh.subscribe(joints_sub);
  
  rev1.attach(Rev1_PIN);
  rev2.attach(Rev2_PIN);
  rev3.attach(Rev3_PIN);
  rev4.attach(Rev4_PIN);
  rev5.attach(Rev5_PIN);
  rev6.attach(GRIPPER_PIN);
  
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
