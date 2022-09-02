#include "base_address_technologies_japan_drivers/rudder.hh"

namespace shipcon::device::base_address_technologies_japan
{
  VectwinRudder::VectwinRudder( std::string node_name, std::string name_space ):
  rclcpp::Node( node_name, name_space )
  {
    getNetworkParam();
    getTopicParam();

    initUdp();
    initPublisher();
    initSubscription();
    initService();
  }

  
  VectwinRudder::~VectwinRudder()
  {
    ;
  }
  

  void VectwinRudder::run( void )
  {
    threadptr_pub_info_ = std::make_unique<std::thread>( &VectwinRudder::thread_receiveUdp, this );
    timerptr_sendcontrol_timer_ = create_wall_timer( sendctrl_rate_, std::bind( &VectwinRudder::thread_sendControl, this ) );
  }


  void VectwinRudder::initUdp( void )
  {
    udp_send_ = std::make_unique<hwcomlib::UdpSend>( device_ip_, device_port_ );
    udp_recv_ = std::make_unique<hwcomlib::UdpRecv>( my_port_ );
  }


  void VectwinRudder::initPublisher( void )
  {
    pub_rudder_info_ = this->create_publisher<shipcon_msgs::msg::RudderInfoTwin>(
      ( std::string( this->get_name() ) + "/rudder_info" ),
      rclcpp::QoS(0)
    );
  }


  void VectwinRudder::initSubscription( void )
  {
    sub_rudder_control_ = this->create_subscription<shipcon_msgs::msg::RudderCommandTwin>(
      subname_rudder_control_,
      rclcpp::QoS(0),
      std::bind( &VectwinRudder::callbackRudderCommand_sub, this, std::placeholders::_1 )
    );
  }


  void VectwinRudder::initService( void )
  {
    srv_get_rudder_spec_ = create_service<shipcon_msgs::srv::GetRudderSpec>(
      ( std::string( this->get_name() ) + "/srv/get_rudder_spec" ),
      std::bind(
        &VectwinRudder::callback_srv_get_rudder_spec,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
      )
    );
  }


  void VectwinRudder::getNetworkParam( void )
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


  void VectwinRudder::getTopicParam( void )
  {
    this->declare_parameter( "topicname/sub/rudder_control" );

    try
    {
      subname_rudder_control_ = this->get_parameter( "topicname/sub/rudder_control" ).as_string();
    }
    catch( rclcpp::exceptions::ParameterNotDeclaredException )
    {
      RCLCPP_ERROR_STREAM( this->get_logger(), "Not set ROS topic parameter!" );
      rclcpp::shutdown();
    }
  }


  void VectwinRudder::thread_sendControl( void )
  {
    udp_send_->sendData( formSendData() );
  }


  void VectwinRudder::thread_receiveUdp( void )
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
        pub_rudder_info_->publish( rudder_info_ );
      }
    }
  }


  void VectwinRudder::updateInfo( std::vector<char>& buffer )
  {
    rudder_info_.header.stamp = this->now();

    char buffer_char[ buffer.size() ];
    for( auto itr = buffer.begin(); itr != buffer.end(); ++itr )
    {
      buffer_char[ std::distance( buffer.begin(), itr ) ] = *itr;
    }

    float rudder_deg_port = 0.0;
    float rudder_deg_stbd = 0.0;
    std::memcpy( &rudder_deg_port, &buffer_char[0], 4 );
    std::memcpy( &rudder_deg_stbd, &buffer_char[4], 4 );

    rudder_info_.port.rudder_angle_rad = static_cast<double>( unitcon::angle::deg2rad ( rudder_deg_port ) );
    rudder_info_.stbd.rudder_angle_rad = static_cast<double>( unitcon::angle::deg2rad ( rudder_deg_stbd ) );
    rudder_info_.port.signal_status.status = shipcon_msgs::msg::SignalStatus::SIGNAL_NORMAL;
    rudder_info_.stbd.signal_status.status = shipcon_msgs::msg::SignalStatus::SIGNAL_NORMAL;
  }


  std::string VectwinRudder::formSendData( void )
  {
    double target_rudder_deg_port, target_rudder_deg_stbd;
    {
      std::lock_guard<std::mutex> lock( mtx_ );
      target_rudder_deg_port = unitcon::angle::rad2deg( rudder_command_.port.target_angle_rad );
      target_rudder_deg_stbd =  unitcon::angle::rad2deg( rudder_command_.stbd.target_angle_rad );
    }

    int16_t rudder_rate_port = static_cast<int16_t>( target_rudder_deg_port * 100.0 / RUDDER_MAX_ANGLE_PORT_DEG );
    int16_t rudder_rate_stbd = static_cast<int16_t>( target_rudder_deg_stbd * 100.0 / RUDDER_MAX_ANGLE_PORT_DEG );

    char buffer[4];
    std::memcpy( &buffer[0], &rudder_rate_port, 2 );
    std::memcpy( &buffer[2], &rudder_rate_stbd, 2 );

    std::string str_buf;
    str_buf.clear();
    for( int cnt = 0; cnt < 4; cnt++ ){ str_buf.push_back( buffer[cnt] ); }

    return str_buf;
  }


  void VectwinRudder::evaluateInfo( void )
  {
    if( rudder_info_.port.rudder_angle_rad > unitcon::angle::deg2rad( RUDDER_MAX_ANGLE_PORT_DEG ) )
    {
      rudder_info_.port.rudder_status.status = shipcon_msgs::msg::RudderHealth::RUDDER_STATUS_OVERANGLE;
    }
    else if( rudder_info_.port.rudder_angle_rad < unitcon::angle::deg2rad( -RUDDER_MAX_ANGLE_STBD_DEG ) )
    {
      rudder_info_.port.rudder_status.status = shipcon_msgs::msg::RudderHealth::RUDDER_STATUS_OVERANGLE;
    }
    else
    {
      rudder_info_.port.rudder_status.status = shipcon_msgs::msg::RudderHealth::RUDDER_STATUS_NORMAL;
    }
    
    if( rudder_info_.stbd.rudder_angle_rad > unitcon::angle::deg2rad( RUDDER_MAX_ANGLE_PORT_DEG ) )
    {
      rudder_info_.stbd.rudder_status.status = shipcon_msgs::msg::RudderHealth::RUDDER_STATUS_OVERANGLE;
    }
    else if( rudder_info_.stbd.rudder_angle_rad < unitcon::angle::deg2rad( -RUDDER_MAX_ANGLE_STBD_DEG ) )
    {
      rudder_info_.stbd.rudder_status.status = shipcon_msgs::msg::RudderHealth::RUDDER_STATUS_OVERANGLE;
    }
    else
    {
      rudder_info_.stbd.rudder_status.status = shipcon_msgs::msg::RudderHealth::RUDDER_STATUS_NORMAL;
    }
  }


  void VectwinRudder::callbackRudderCommand_sub( const shipcon_msgs::msg::RudderCommandTwin::SharedPtr msg )
  {
    std::lock_guard<std::mutex> lock( mtx_ );
    rudder_command_.header = msg->header;
    if( msg->port.target_angle_rad > RUDDER_MAX_ANGLE_PORT_DEG )
    {
      rudder_command_.port.target_angle_rad = RUDDER_MAX_ANGLE_PORT_DEG;
    }
    else if( msg->port.target_angle_rad < -RUDDER_MAX_ANGLE_STBD_DEG )
    {
      rudder_command_.port.target_angle_rad = -RUDDER_MAX_ANGLE_STBD_DEG;
    }
    else
    {
      rudder_command_.port.target_angle_rad = msg->port.target_angle_rad;
    }

    if( msg->stbd.target_angle_rad > RUDDER_MAX_ANGLE_PORT_DEG )
    {
      rudder_command_.stbd.target_angle_rad = RUDDER_MAX_ANGLE_PORT_DEG;
    }
    else if( msg->stbd.target_angle_rad < -RUDDER_MAX_ANGLE_STBD_DEG )
    {
      rudder_command_.stbd.target_angle_rad = -RUDDER_MAX_ANGLE_STBD_DEG;
    }
    else
    {
      rudder_command_.stbd.target_angle_rad = msg->stbd.target_angle_rad;
    }
  }


  void VectwinRudder::callback_srv_get_rudder_spec(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<shipcon_msgs::srv::GetRudderSpec_Request> req,
    const std::shared_ptr<shipcon_msgs::srv::GetRudderSpec_Response> res
  )
  {
    res->developper = DEVELOPPER_NAME;
    res->type = DEVICE_TYPE;
    res->ip = device_ip_;
    res->netmask = device_ip_mask_;
    res->max_angle = RUDDER_MAX_ANGLE_PORT_DEG;
    res->min_angle = RUDDER_MAX_ANGLE_STBD_DEG;
  }
}