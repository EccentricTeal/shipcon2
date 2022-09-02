#include "base_address_technologies_japan_drivers/radio_receiver.hh"

int main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  auto rc_node = std::make_shared<shipcon::device::base_address_technologies_japan::VectwinRadioReceiver>( "VectwinRadioReceiver", "/shipcon/device" );
  rc_node->run();
  rclcpp::spin( rc_node );

  rclcpp::shutdown();
}