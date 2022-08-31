#include "base_address_technologies_japan_drivers/motor.hh"

namespace shipcon::device::base_address_technologies_japan
{
  CppMotor::CppMotor( std::string node_name, std::string name_space ):
  rclcpp::Node( node_name, name_space )
  {
    getNetworkParam();
    getTopicParam();

    initUdp();
    initPublisher();
    initSubscription();
  }

  
  CppMotor::~CppMotor()
  {
    ;
  }
  

  void CppMotor::run( void )
  {
    timerptr_pubinfo_timer_ = create_wall_timer( pubinfo_rate_, std::bind( &CppMotor::thread_receiveUdp, this ) );
    timerptr_sendcontrol_timer_ = create_wall_timer( sendctrl_rate_, std::bind( &CppMotor::thread_sendControl, this ) );
  }


  void CppMotor::initUdp( void )
  {
    udp_send_ = std::make_unique<hwcomlib::UdpSend>( device_ip_, device_port_ );
    udp_recv_ = std::make_unique<hwcomlib::UdpRecv>( my_port_ );
  }


  void CppMotor::initPublisher( void )
  {
    pub_motor_info_ = this->create_publisher<shipcon_msgs::msg::MotorInfo>(
      "motor_info",
      rclcpp::QoS(0)
    );

    pub_prop_info_ = this->create_publisher<shipcon_msgs::msg::PropellerInfo>(
      "propeller_info",
      rclcpp::QoS(0)
    );
  }


  void CppMotor::initSubscription( void )
  {
    sub_motor_control_ = this->create_subscription<shipcon_msgs::msg::MotorCommand>(
      subname_motor_control_,
      rclcpp::QoS(0),
      std::bind( &CppMotor::callbackMotorCommand_sub_, this, std::placeholders::_1 )
    );

    sub_prop_control_ = this->create_subscription<shipcon_msgs::msg::PropellerCommand>(
      subname_prop_control_,
      rclcpp::QoS(0),
      std::bind( &CppMotor::callbackPropellerCommand_sub_, this, std::placeholders::_1 )
    );
  }


  void CppMotor::getNetworkParam( void )
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


  void CppMotor::getTopicParam( void )
  {
    this->declare_parameter( "topicname/sub/motor_control" );
    this->declare_parameter( "topicname/sub/propeller_control" );

    try
    {
      subname_motor_control_ = this->get_parameter( "topicname/sub/motor_control" ).as_string();
      subname_prop_control_ = this->get_parameter( "topicname/sub/propeller_control" ).as_string();
    }
    catch( rclcpp::exceptions::ParameterNotDeclaredException )
    {
      RCLCPP_ERROR_STREAM( this->get_logger(), "Not set ROS topic parameter!" );
      rclcpp::shutdown();
    }
  }


  void CppMotor::thread_sendControl( void )
  {
    udp_send_->sendData( formSendData() );
  }


  void CppMotor::thread_receiveUdp( void )
  {
    std::vector<char> buffer;
    buffer.clear();
    auto recvdata = udp_recv_->recvData( device_ip_, buffer );

    if( std::get<0>( recvdata ) == DATASIZE )
    {
      updateInfo( buffer );
      evaluateInfo();
      pub_motor_info_->publish( motor_info_ );
      pub_prop_info_->publish( prop_info_ );
    }
  }


  void CppMotor::updateInfo( std::vector<char>& buffer )
  {
    motor_info_.header.stamp = this->now();
    prop_info_.header.stamp = this->now();

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

    motor_info_.rotspeed_rpm = static_cast<double>( motor_rpm );
    motor_info_.current_amp = static_cast<double>( motor_current );
    motor_info_.signal_status.status = shipcon_msgs::msg::SignalStatus::SIGNAL_NORMAL;

    prop_info_.prop_pitch_rad = static_cast<double>( unitcon::angle::deg2rad ( prop_pitch_rad ) );
    prop_info_.signal_status.status = shipcon_msgs::msg::SignalStatus::SIGNAL_NORMAL;
  }


  std::string CppMotor::formSendData( void )
  {
    double target_motor_rpm, target_prop_rad;
    {
      std::lock_guard<std::mutex> lock( mtx_ );
      target_motor_rpm = motor_command_.target_speed_rpm;
      target_prop_rad = prop_command_.target_prop_pitch_rad;
    }

    int16_t motor_rate = static_cast<int16_t>( target_motor_rpm * 100.0 / MOTOR_MAX_RPM );
    int16_t prop_rate = static_cast<int16_t>( target_prop_rad * 100.0 / unitcon::angle::deg2rad( PROP_MAX_ANGLE_AHEAD_DEG ) );

    char buffer[4];
    std::memcpy( &buffer[0], &motor_rate, 2 );
    std::memcpy( &buffer[2], &prop_rate, 2 );

    std::string str_buf;
    str_buf.clear();
    for( int cnt = 0; cnt < 4; cnt++ ){ str_buf.push_back( buffer[cnt] ); }

    return str_buf;
  }


  void CppMotor::evaluateInfo( void )
  {
    motor_info_.motor_status.status = shipcon_msgs::msg::MotorHealth::STATUS_NORMAL;
    if( std::abs( motor_info_.rotspeed_rpm ) > MOTOR_MAX_RPM )
    {
      motor_info_.motor_status.status = shipcon_msgs::msg::MotorHealth::STATUS_OVERSPEED;
    }

    prop_info_.prop_status.status = shipcon_msgs::msg::PropellerHealth::STATUS_NORMAL;
    if( prop_info_.prop_pitch_rad > unitcon::angle::deg2rad( PROP_MAX_ANGLE_AHEAD_DEG ) )
    {
      prop_info_.prop_status.status = shipcon_msgs::msg::PropellerHealth::STATUS_OVERANGLE_AHEAD;
    }
    if( prop_info_.prop_pitch_rad < unitcon::angle::deg2rad( -PROP_MAX_ANGLE_ASTERN_DEG ) )
    {
      prop_info_.prop_status.status = shipcon_msgs::msg::PropellerHealth::STATUS_OVERANGLE_ASTERN;
    }
  }


  void CppMotor::callbackMotorCommand_sub_( const shipcon_msgs::msg::MotorCommand::SharedPtr msg )
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


  void CppMotor::callbackPropellerCommand_sub_( const shipcon_msgs::msg::PropellerCommand::SharedPtr msg )
  {
    std::lock_guard<std::mutex> lock( mtx_ );
    prop_command_.header = msg->header;
    if( msg->target_prop_pitch_rad > unitcon::angle::deg2rad( PROP_MAX_ANGLE_AHEAD_DEG ) ) 
    {
      prop_command_.target_prop_pitch_rad = PROP_MAX_ANGLE_AHEAD_DEG;
    }
    else if( msg->target_prop_pitch_rad < unitcon::angle::deg2rad( -PROP_MAX_ANGLE_ASTERN_DEG ) )
    {
      prop_command_.target_prop_pitch_rad = -PROP_MAX_ANGLE_ASTERN_DEG;
    }
    else
    {
      prop_command_.target_prop_pitch_rad = msg->target_prop_pitch_rad;
    }
  }
}