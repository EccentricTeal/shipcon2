#include "base_address_technologies_japan_drivers/motor.hh"

int main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  shipcon::device::base_address_technologies_japan::CppMotor motor_node( "CppMotor", "/shipcon/device" );
  motor_node.run();

  rclcpp::shutdown();
}