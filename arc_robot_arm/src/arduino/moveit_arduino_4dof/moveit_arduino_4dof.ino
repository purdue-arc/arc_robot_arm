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

float gripper;
float arm[DOF];

void set_gripper_joints(const sensor_msgs::JointState& joint_state){
  gripper = joint_state.position[1]; // left finger actuation only
}

void set_arm_joints(const sensor_msgs::JointState& joint_state) {
  arm[0] = joint_state.position[0];
  arm[1] = joint_state.position[1];
  arm[2] = joint_state.position[2];
//arm[3] = joint_state.position[3];
  arm[4] = joint_state.position[3];
}

int is_arm_move_group(const sensor_msgs::JointState& joint_state){
  int pos_array_len = sizeof(joint_state.position)/sizeof(joint_state.position[0]);
  return pos_array_len == 4;
}

void print_arr(float arr[]){
  int arr_size = sizeof(arr) / sizeof(arr[0]);
  char result[8]; // Buffer big enough for 7-character float
  for(int i = 0; i < arr_size; i++) {
    dtostrf(arr[i], 6, 2, result); // Leave room for too large numbers!
    nh.loginfo(result);
  }
}

void joint_control_cb( const sensor_msgs::JointState& joint_state) { // output can be either for gripper joint or for arm joints

  if(is_arm_move_group(joint_state))  {
    set_arm_joints(joint_state);
    nh.logdebug("arm");
    print_arr(arm);
  }
  else{
    set_gripper_joints(joint_state);
    nh.logdebug("gripper");
  }
  nh.logdebug("Recieved");
 //servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
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
}

void loop(){
  nh.spinOnce();
  delay(1);
}
