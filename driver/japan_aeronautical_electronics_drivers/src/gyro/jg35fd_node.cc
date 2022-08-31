#include "japan_aeronautical_electronics_drivers/gyro.hh"
#include "rclcpp/rclcpp.hpp"

int main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  auto gyro_node = std::make_shared<shipcon::device::japan_aeronautical_electronics::GyroJg35fd>( "GyroJG35FD", "/shipcon/device" );
  rclcpp::spin( gyro_node );
  rclcpp::shutdown();
}