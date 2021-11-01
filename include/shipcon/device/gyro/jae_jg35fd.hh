#ifndef SHIPCON__DEVICE__GYRO__JAE_JG35FD_HH
#define SHIPCON__DEVICE__GYRO__JAE_JG35FD_HH

#include <rclcpp/rclcpp.hpp>
#include "hardware_communication_lib/serialcom.hh"

namespace shipcon::device
{
  class GyroJaeJG35FD
  {
    public:
      GyroJaeJG35FD();
      ~GyroJaeJG35FD();
  };
}

#endif