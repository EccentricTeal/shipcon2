#include "base_address_technologies_japan_drivers/rudder.hh"

int main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  auto rudder_node = std::make_shared<shipcon::device::base_address_technologies_japan::VectwinRudder>( "VectwinRudder", "/shipcon/device" );
  rudder_node->run();
  rclcpp::spin( rudder_node );

  rclcpp::shutdown();
}