#include "base_address_technologies_japan_drivers/motor.hh"

int main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  auto motor_node = std::make_shared<shipcon::device::base_address_technologies_japan::CppMotor>( "CppMotor", "/shipcon/device" );
  motor_node->run();
  rclcpp::spin( motor_node );

  rclcpp::shutdown();
}