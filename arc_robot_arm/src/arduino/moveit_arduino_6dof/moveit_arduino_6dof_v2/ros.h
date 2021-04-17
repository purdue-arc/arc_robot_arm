#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "ArduinoHardware.h"

namespace ros
{
  // default is 25, 25, 512, 512  
  typedef NodeHandle_<ArduinoHardware, 10, 15, 128, 256> NodeHandle;

  // This is legal too and will use the default 25, 25, 512, 512
  //typedef NodeHandle_<ArduinoHardware> NodeHandle;
}

#endif
