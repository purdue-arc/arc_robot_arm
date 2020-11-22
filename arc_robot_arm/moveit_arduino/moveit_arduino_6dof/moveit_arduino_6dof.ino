/*
 * rosserial Servo Control Example
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

#define J1 11
#define J2 10
#define J3 9
#define J4 6
#define J5 5
#define J6 4

ros::NodeHandle  nh;

Servo servo;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
}


ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup(){
  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(J1); //attach to motor at pin J1
}

void loop(){
  nh.spinOnce();
  delay(1);
}



