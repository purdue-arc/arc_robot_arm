/*
 * Rosserial driver for the protoarm
 */
#include <protoarm.h>

ros::NodeHandle  nh;

void robot_cb(const sensor_msgs::JointState& joint_state) {
  if(joint_state.name[0][0] == 'G') { // Gripper command
    protoarm::drive_gripper(joint_state, nh);
  }
  else {
    protoarm::drive_arm(joint_state, nh);
  }
  //protoarm::log(nh);
}

ros::Subscriber<sensor_msgs::JointState> robot_sub("/move_group/fake_controller_joint_states", robot_cb);

void setup() {
  nh.initNode();
  nh.subscribe(robot_sub);
  protoarm::init();
}

void loop() {
  nh.spinOnce();
  delay(10);
}
