/*
 * Rosserial driver for the protoarm
 */
#include <protoarm_driver/protoarm.h>

ros::NodeHandle  nh;

void arm_cb(const sensor_msgs::JointState& joint_state) {
  protoarm::move_arm(joint_state, nh);
  //protoarm::log(nh);
}
void gripper_cb(const sensor_msgs::JointState& joint_state) {
  protoarm::move_gripper(joint_state, nh);
  //protoarm::log(nh);
}

ros::Subscriber<sensor_msgs::JointState> arm_sub("/arm_controller/state", arm_cb);
ros::Subscriber<sensor_msgs::JointState> gripper_sub("/gripper_controller/state", gripper_cb);

void setup() {
  Serial.begin(4800);
  nh.initNode();
  nh.subscribe(arm_sub);
  nh.subscribe(gripper_sub);
  protoarm::init();
}

void loop() {
  nh.spinOnce();
  delay(1);
}
