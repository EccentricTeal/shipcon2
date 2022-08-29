#include "base_address_technologies_japan_drivers/motor.hh"

namespace shipcon::device::base_address_technologies_japan
{
  CppMotor::CppMotor( std::string node_name, std::string name_space ):
  rclcpp::Node( node_name, name_space )
  {
    getNetworkParam();
    udp_send_ = std::make_unique<hwcomlib::UdpSend>( device_ip_, device_port_ );
    udp_recv_ = std::make_unique<hwcomlib::UdpRecv>( my_port_ );

    
  }

  
  CppMotor::~CppMotor()
  {
    if( threadptr_update_info_->joinable() ){ threadptr_update_info_->join(); }
  }
  

  void CppMotor::run( void )
  {
    threadptr_update_info_ = std::make_unique<std::thread>( &CppMotor::thread_receiveUdp, this );
  }


  void CppMotor::getNetworkParam( void )
  {
    try
    {
      device_ip_ = ( this->get_parameter( "device_ip" ) ).as_string();
      my_port_ = static_cast<uint16_t>( ( this->get_parameter( "my_port" ) ).as_int() );
      device_port_ = static_cast<uint16_t>( ( this->get_parameter( "device_port" ) ).as_int() );
    }
    catch( rclcpp::exceptions::ParameterNotDeclaredException )
    {
      RCLCPP_ERROR_STREAM( this->get_logger(), "Not set network parameter!" );
      std::terminate();
    }
  }


  void CppMotor::thread_receiveUdp( void )
  {
    while( rclcpp::ok() )
    {
      std::vector<char> buffer;
      buffer.clear();
      auto recvdata = udp_recv_->recvData( device_ip_, buffer );

      if( std::get<0>( recvdata ) == DATASIZE )
      {
        char buffer_char[ buffer.size() ];
        for( auto itr = buffer.begin(); itr != buffer.end(); ++itr )
        {
          buffer_char[ std::distance( buffer.begin(), itr ) ] = *itr;
        }

        int16_t motor_rpm = 0;
        float motor_current = 0.0;
        float prop_pitch_rad = 0.0;
        std::memcpy( &motor_rpm, &buffer_char[0], 2 );
        std::memcpy( &motor_current, &buffer_char[2], 4 );
        std::memcpy( &prop_pitch_rad, &buffer_char[6], 4 );

        std::lock_guard<std::mutex> lock( mtx_ );
        motor_info_.rotspeed_rpm = static_cast<double>( motor_rpm );
        motor_info_.current_amp = static_cast<double>( motor_current );
      }
    }
  }
}