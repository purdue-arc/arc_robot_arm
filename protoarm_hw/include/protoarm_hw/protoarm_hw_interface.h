#ifndef PROTOARM_HW_INTERFACE
#define PROTOARM_HW_INTERFACE

#include <ros_control_boilerplate/generic_hw_interface.h>

namespace protoarm_hw {

  class ProtoarmHWInterface : public ros_control_boilerplate::GenericHWInterface {
    public:
    /**
     * \brief Constructor
     * \param nh - Node handle for topics.
     */
    ProtoarmHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

    /** \brief Read the state from the robot hardware. */
    virtual void read(ros::Duration& elapsed_time);

    /** \brief Write the command to the robot hardware. */
    virtual void write(ros::Duration& elapsed_time);

    /** \brief Enforce limits for all values before writing */
    virtual void enforceLimits(ros::Duration& period);
  };

} 
#endif
