#include "base_address_technologies_japan_drivers/motor.hh"

namespace shipcon::device::base_address_technologies_japan
{
  NormalMotor::NormalMotor( std::string node_name, std::string name_space ):
  rclcpp::Node( node_name, name_space )
  {
    getNetworkParam();
    getTopicParam();

    initUdp();
    initPublisher();
    initSubscription();
    initService();
  }

  
  NormalMotor::~NormalMotor()
  {
    ;
  }
  

  void NormalMotor::run( void )
  {
    threadptr_pub_info_ = std::make_unique<std::thread>( &NormalMotor::thread_receiveUdp, this );
    timerptr_sendcontrol_timer_ = create_wall_timer( sendctrl_rate_, std::bind( &NormalMotor::thread_sendControl, this ) );
  }


  void NormalMotor::initUdp( void )
  {
    udp_send_ = std::make_unique<hwcomlib::UdpSend>( device_ip_, device_port_ );
    udp_recv_ = std::make_unique<hwcomlib::UdpRecv>( my_port_ );
  }


  void NormalMotor::initPublisher( void )
  {
    pub_motor_info_ = this->create_publisher<shipcon_msgs::msg::MotorInfo>(
      ( std::string( this->get_name() ) + "/motor_info" ),
      rclcpp::QoS(0)
    );
  }


  void NormalMotor::initSubscription( void )
  {
    sub_motor_control_ = this->create_subscription<shipcon_msgs::msg::MotorCommand>(
      subname_motor_control_,
      rclcpp::QoS(0),
      std::bind( &NormalMotor::callbackMotorCommand_sub_, this, std::placeholders::_1 )
    );
  }


  void NormalMotor::initService( void )
  {
    srv_get_motor_spec_ = create_service<shipcon_msgs::srv::GetMotorSpec>(
      ( std::string( this->get_name() ) + "/srv/get_motor_spec" ),
      std::bind(
        &NormalMotor::callback_srv_get_motor_spec,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
      )
    );
  }


  void NormalMotor::getNetworkParam( void )
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


  void NormalMotor::getTopicParam( void )
  {
    this->declare_parameter( "topicname/sub/motor_control" );
    this->declare_parameter( "topicname/sub/propeller_control" );

    try
    {
      subname_motor_control_ = this->get_parameter( "topicname/sub/motor_control" ).as_string();
    }
    catch( rclcpp::exceptions::ParameterNotDeclaredException )
    {
      RCLCPP_ERROR_STREAM( this->get_logger(), "Not set ROS topic parameter!" );
      rclcpp::shutdown();
    }
  }


  void NormalMotor::thread_sendControl( void )
  {
    udp_send_->sendData( formSendData() );
  }


  void NormalMotor::thread_receiveUdp( void )
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
        pub_motor_info_->publish( motor_info_ );
      }
    }
  }


  void NormalMotor::updateInfo( std::vector<char>& buffer )
  {
    motor_info_.header.stamp = this->now();

    char buffer_char[ buffer.size() ];
    for( auto itr = buffer.begin(); itr != buffer.end(); ++itr )
    {
      buffer_char[ std::distance( buffer.begin(), itr ) ] = *itr;
    }

    int16_t motor_rpm = 0;
    float motor_current = 0.0;
    float motor_temperature = 0.0;
    std::memcpy( &motor_rpm, &buffer_char[0], 2 );
    std::memcpy( &motor_current, &buffer_char[2], 4 );
    std::memcpy( &motor_temperature, &buffer_char[6], 4 );

    motor_info_.rotspeed_rpm = static_cast<double>( motor_rpm );
    motor_info_.current_amp = static_cast<double>( motor_current );
    motor_info_.temprature_celdeg = static_cast<double>( motor_temperature );
    motor_info_.signal_status.status = shipcon_msgs::msg::SignalStatus::SIGNAL_NORMAL;
  }


  std::string NormalMotor::formSendData( void )
  {
    double target_motor_rpm, target_prop_rad;
    {
      std::lock_guard<std::mutex> lock( mtx_ );
      target_motor_rpm = motor_command_.target_speed_rpm;
    }

    int16_t motor_rate = static_cast<int16_t>( target_motor_rpm * 100.0 / MOTOR_MAX_RPM );

    char buffer[2];
    std::memcpy( &buffer[0], &motor_rate, 2 );

    std::string str_buf;
    str_buf.clear();
    for( int cnt = 0; cnt < 2; cnt++ ){ str_buf.push_back( buffer[cnt] ); }

    return str_buf;
  }


  void NormalMotor::evaluateInfo( void )
  {
    motor_info_.motor_status.status = shipcon_msgs::msg::MotorHealth::STATUS_NORMAL;
    if( std::abs( motor_info_.rotspeed_rpm ) > MOTOR_MAX_RPM )
    {
      motor_info_.motor_status.status = shipcon_msgs::msg::MotorHealth::STATUS_OVERSPEED;
    }
  }


  void NormalMotor::callbackMotorCommand_sub_( const shipcon_msgs::msg::MotorCommand::SharedPtr msg )
  {
    std::lock_guard<std::mutex> lock( mtx_ );
    motor_command_.header = msg->header;
    if( msg->target_speed_rpm > MOTOR_MAX_RPM )
    {
      motor_command_.target_speed_rpm = MOTOR_MAX_RPM;
    }
    else if( msg->target_speed_rpm < -MOTOR_MAX_RPM )
    {
      motor_command_.target_speed_rpm = -MOTOR_MAX_RPM;
    }
    else
    {
      motor_command_.target_speed_rpm = msg->target_speed_rpm;
    }
  }


  void NormalMotor::callback_srv_get_motor_spec(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<shipcon_msgs::srv::GetMotorSpec_Request> req,
    const std::shared_ptr<shipcon_msgs::srv::GetMotorSpec_Response> res
  )
  {
    res->developper = DEVELOPPER_NAME;
    res->type = DEVICE_TYPE;
    res->ip = device_ip_;
    res->netmask = device_ip_mask_;
    res->max_speed = MOTOR_MAX_RPM;
  }
}