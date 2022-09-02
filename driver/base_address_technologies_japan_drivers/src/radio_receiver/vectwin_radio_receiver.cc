#include "base_address_technologies_japan_drivers/radio_receiver.hh"

namespace shipcon::device::base_address_technologies_japan
{
  VectwinRadioReceiver::VectwinRadioReceiver( std::string node_name, std::string name_space ):
  rclcpp::Node( node_name, name_space )
  {
    getNetworkParam();

    initUdp();
    initPublisher();
  }

  
  VectwinRadioReceiver::~VectwinRadioReceiver()
  {
    ;
  }
  

  void VectwinRadioReceiver::run( void )
  {
    threadptr_pub_info_ = std::make_unique<std::thread>( &VectwinRadioReceiver::thread_receiveUdp, this );
  }


  void VectwinRadioReceiver::initUdp( void )
  {
    udp_send_ = std::make_unique<hwcomlib::UdpSend>( device_ip_, device_port_ );
    udp_recv_ = std::make_unique<hwcomlib::UdpRecv>( my_port_ );
  }


  void VectwinRadioReceiver::initPublisher( void )
  {
    pub_status_ = this->create_publisher<base_address_technologies_japan_msgs::msg::RadioReceiverStatus>(
      ( std::string( this->get_name() ) + "/device_status" ),
      rclcpp::QoS(0)
    );

    pub_ship_mode_ = this->create_publisher<base_address_technologies_japan_msgs::msg::RadioReceiverShipMode>(
      ( std::string( this->get_name() ) + "/ship_mode" ),
      rclcpp::QoS(0)
    );

    pub_vectwin_mode_ = this->create_publisher<base_address_technologies_japan_msgs::msg::RadioReceiverVectwinMode>(
      ( std::string( this->get_name() ) + "/vectwin_mode" ),
      rclcpp::QoS(0)
    );

    pub_joy_ = this->create_publisher<sensor_msgs::msg::Joy>(
      ( std::string( this->get_name() ) + "/radio_control_joy_info" ),
      rclcpp::QoS(0)
    );
  }


  void VectwinRadioReceiver::getNetworkParam( void )
  {
    this->declare_parameter( "network/device_ip" );
    this->declare_parameter( "network/device_ip_mask" );
    this->declare_parameter( "network/my_port" );
    this->declare_parameter( "network/device_port" );

    try
    {
      device_ip_ = this->get_parameter( "network/device_ip" ).as_string();
      device_ip_mask_ = this->get_parameter( "network/device_ip_mask" ).as_string();
      my_port_ = static_cast<uint16_t>( this->get_parameter( "network/my_port" ).as_int() );
      device_port_ = static_cast<uint16_t>( this->get_parameter( "network/device_port" ).as_int() );
    }
    catch( rclcpp::exceptions::ParameterNotDeclaredException )
    {
      RCLCPP_ERROR_STREAM( this->get_logger(), "Not set network parameter!" );
      rclcpp::shutdown();
    }
  }


  void VectwinRadioReceiver::thread_receiveUdp( void )
  {
    while( rclcpp::ok() )
    {
      std::vector<char> buffer;
      buffer.clear();
      auto recvdata = udp_recv_->recvData( device_ip_, buffer );

      if( std::get<0>( recvdata ) == DATASIZE )
      {
        updateInfo( buffer );
        evaluateInfo();
        pub_status_->publish( status_ );
        pub_ship_mode_->publish( ship_mode_ );
        pub_vectwin_mode_->publish( vectwin_mode_ );
        pub_joy_->publish( joy_ );
      }
    }
  }


  void VectwinRadioReceiver::updateInfo( std::vector<char>& buffer )
  {
    auto timestamp = this->now();
    status_.header.stamp = timestamp;
    ship_mode_.header.stamp = timestamp;
    vectwin_mode_.header.stamp = timestamp;
    joy_.header.stamp = timestamp;

    char buffer_char[ buffer.size() ];
    for( auto itr = buffer.begin(); itr != buffer.end(); ++itr )
    {
      buffer_char[ std::distance( buffer.begin(), itr ) ] = *itr;
    }

  }


  void VectwinRadioReceiver::evaluateInfo( void )
  {
    status_.status = base_address_technologies_japan_msgs::msg::RadioReceiverStatus::STATUS_NORMAL;
  }
}