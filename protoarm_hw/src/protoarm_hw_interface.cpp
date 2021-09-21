#include <protoarm_hw/protoarm_hw_interface.h>

namespace protoarm_hw {

  ProtoarmHWInterface::ProtoarmHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
  {
    ROS_INFO_NAMED("protoarm_hw_interface", "ProtoarmHWInterface Ready.");
  }

  void ProtoarmHWInterface::read(ros::Duration& elapsed_time)
  {
    
  }

  void ProtoarmHWInterface::write(ros::Duration& elapsed_time)
  {
    // Safety
    enforceLimits(elapsed_time);

    // DUMMY PASS-THROUGH CODE
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
      joint_position_[joint_id] += joint_position_command_[joint_id];
  }

  void ProtoarmHWInterface::enforceLimits(ros::Duration& period)
  {
    pos_jnt_sat_interface_.enforceLimits(period);
  }

}
