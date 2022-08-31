#include "japan_aeronautical_electronics_drivers/gyro.hh"


namespace shipcon::device::japan_aeronautical_electronics
{
  GyroJg35fd::GyroJg35fd( std::string node_name, std::string name_space ):
  rclcpp::Node( node_name, name_space )
  {
    //init ROS Service
    initServiceParameter();
    initService();

    //init serial device
    if ( initSerial() ){ serialif_->run(); }

    //init data variables
    data_buffer_.clear();
    yaw_angle_ = 0.0;

    serialif_->dispatchRecvUntil(
      recv_buffer_,
      REGEX_CONDITION_HEADER,
      std::bind(
        &GyroJg35fd::callback_receive_header,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );
  }

  GyroJg35fd::~GyroJg35fd()
  {
    ;
  }


  void GyroJg35fd::initServiceParameter( void )
  {
    this->declare_parameter( "servicename/control_output" );
    this->declare_parameter( "servicename/calibrate_bias_drift" );
    this->declare_parameter( "servicename/control_calculate" );
    this->declare_parameter( "servicename/reset_angle" );
    this->declare_parameter( "servicename/set_analog_mode" );

    try
    {
      srvname_control_output_ = this->get_parameter( "servicename/control_output" ).as_string();
      srvname_calibrate_bias_drift_ = this->get_parameter( "servicename/calibrate_bias_drift" ).as_string();
      srvname_control_calculate_ = this->get_parameter( "servicename/control_calculate" ).as_string();
      srvname_reset_angle_ = this->get_parameter( "servicename/reset_angle" ).as_string();
      srvname_set_analog_mode_ = this->get_parameter( "servicename/set_analog_mode" ).as_string();
    }
    catch( rclcpp::exceptions::ParameterNotDeclaredException )
    {
      RCLCPP_ERROR_STREAM( this->get_logger(), "Not set Service name in parameter!" );
      rclcpp::shutdown();
    }
  }


  void GyroJg35fd::initService( void )
  {
    srv_control_output_ = create_service<japan_aeronautical_electronics_msgs::srv::Jg35fdControlOutput>(
      srvname_control_output_,
      std::bind(
        &GyroJg35fd::callback_srv_control_output,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
      )
    );
    
    srv_calibrate_bias_drift_ = create_service<japan_aeronautical_electronics_msgs::srv::Jg35fdCalibrateBiasDrift>(
      srvname_calibrate_bias_drift_,
      std::bind(
        &GyroJg35fd::callback_srv_calibrate_bias_drift,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
      )
    );
  
    srv_control_calculate_ = create_service<japan_aeronautical_electronics_msgs::srv::Jg35fdControlCalculate>(
      srvname_control_calculate_,
      std::bind(
        &GyroJg35fd::callback_srv_control_calculate,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
      )
    );
    
    srv_reset_angle_ = create_service<japan_aeronautical_electronics_msgs::srv::Jg35fdResetAngle>(
      srvname_reset_angle_,
      std::bind(
        &GyroJg35fd::callback_srv_reset_angle,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
      )
    );
    
    srv_set_analog_mode_ = create_service<japan_aeronautical_electronics_msgs::srv::Jg35fdSetAnalogMode>(
      srvname_set_analog_mode_,
      std::bind(
        &GyroJg35fd::callback_srv_set_analog_mode,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
      )
    );
  }


  void GyroJg35fd::initSerialParameter( void )
  {
    this->declare_parameter( "serial/port_name" );

    try
    {
      serial_port_name_ = this->get_parameter( "serial/port_name" ).as_string();
    }
    catch( rclcpp::exceptions::ParameterNotDeclaredException )
    {
      RCLCPP_ERROR_STREAM( this->get_logger(), "Not set Serial Port as parameter!" );
      rclcpp::shutdown();
    }
  }


  bool GyroJg35fd::initSerial( void )
  {  
    serialif_ = std::make_unique<hwcomlib::SerialCom>( serial_port_name_, BAUDRATE );

    if( serialif_ )
    {
      serialif_->setCharacterSize( 8 );
      serialif_->setFlowControl( boost::asio::serial_port_base::flow_control::none );
      serialif_->setParity( boost::asio::serial_port_base::parity::odd );
      serialif_->setStopBits( boost::asio::serial_port_base::stop_bits::one );
      return true;
    }
    return false;
  }


  void GyroJg35fd::callback_sendSerial( const boost::system::error_code& ec, std::size_t sendsize )
  {
    //std::cout << std::dec << sendsize << std::endl;
  }


  void GyroJg35fd::callback_receive_header( const boost::system::error_code& ec, std::size_t recvsize )
  {
    std::istream istr( &recv_buffer_ );

    //Set header data(1 or 2 bytes) from istream 
    if( recvsize == 1 )
    {
      data_buffer_.push_back( istr.get() );
    }
    else
    {
      std::vector<unsigned char> temp;
      temp.clear();
      while( istr ){ temp.push_back( istr.get() ); }
      
      data_buffer_.clear();
      data_buffer_.push_back( *( temp.end() - 3 ) ); //0x02
      data_buffer_.push_back( *( temp.end() - 2 ) ); //0x81-0x84
      //Last istream byte( temp.end()-1 ) is 0xff
    }
    
    //Receiving remaining data packet
    //0x81, 0x82 or 0x84 message contains 2byte data
    if( data_buffer_[1] == 0x81 || data_buffer_[1] == 0x82 || data_buffer_[1] == 0x84 )
    {
      serialif_->dispatchRecvSize(
        recv_buffer_,
        5,
        std::bind(
          &GyroJg35fd::callback_receive_data,
          this,
          std::placeholders::_1,
          std::placeholders::_2,
          2
        )
      );
    }
    //0x83 message contains 4byte data
    else if( data_buffer_[1] == 0x83 )
    {
      serialif_->dispatchRecvSize(
        recv_buffer_,
        7,
        std::bind(
          &GyroJg35fd::callback_receive_data,
          this,
          std::placeholders::_1,
          std::placeholders::_2,
          4
        )
      );
    }
    //If next 0x02 data is not 0x81-0x84, this 0x02 is not indicator of STX of header
    //So, read until header again. System will enter to this case only when it face error.
    else
    {
      serialif_->dispatchRecvUntil(
      recv_buffer_,
      REGEX_CONDITION_HEADER,
      std::bind(
        &GyroJg35fd::callback_receive_header,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );
    }
  }


  void GyroJg35fd::callback_receive_data( const boost::system::error_code& ec, std::size_t recvsize, unsigned int datasize )
  {
    std::istream istr( &recv_buffer_ );
        
    //This callback functions receive only 5 or 7 bytes normally.
    //In case receiving the other number of data, it is error, so going back to header receive process
    if( recvsize != datasize + 3 )
    {
      serialif_->dispatchRecvUntil(
        recv_buffer_,
        REGEX_CONDITION_HEADER,
        std::bind(
          &GyroJg35fd::callback_receive_header,
          this,
          std::placeholders::_1,
          std::placeholders::_2
        )
      );
      return;
    }

    //receive data (discard final 0xff of istream)
    for( int cnt = 2; cnt < ( 2 + recvsize ); cnt++ ){ data_buffer_.push_back( istr.get() ); }
    recv_buffer_.consume( recv_buffer_.size() );

    //Evaluate final byte whether 0x0d or not
    //In case the header got previous process is true header, update data variable.
    if( *( std::prev( data_buffer_.end() ) ) == 0x0d )//in case received header is true header
    {  
      updateData();
    }
    //In case received header is just a part of data
    else
    {
      //Find next header 0x02 + 0x81-0x84
      auto itr = data_buffer_.begin()+2; //From next "not true header"
      for( ; itr != data_buffer_.end(); ++itr )
      {
        itr = std::find( itr, data_buffer_.end(), 0x02 );

        //Not found
        if( itr == data_buffer_.end() )
        {
          serialif_->dispatchRecvUntil(
            recv_buffer_,
            REGEX_CONDITION_HEADER,
            std::bind(
              &GyroJg35fd::callback_receive_header,
              this,
              std::placeholders::_1,
              std::placeholders::_2
            )
          );
          break;
        }
        //Found 0x02 in the end of buffer
        else if( itr == data_buffer_.end() - 1 )
        {
          data_buffer_.clear();
          data_buffer_.push_back( *itr );
          serialif_->dispatchRecvSize(
            recv_buffer_,
            1,
            std::bind(
              &GyroJg35fd::callback_receive_header,
              this,
              std::placeholders::_1,
              std::placeholders::_2
            )
          );
          break;
        }
        //Found 0x02 not in the end of buffer
        else
        {
          if( *( std::next( itr ) ) == 0x81 || *( std::next( itr ) ) == 0x82 || *( std::next( itr ) ) == 0x84 )
          {
            std::vector<unsigned char> tempv;
            tempv.clear();
            for( ; itr != data_buffer_.end(); ++itr )
            {
              tempv.push_back( *itr );
            }
            data_buffer_.clear();
            std::copy( tempv.begin(), tempv.end(), data_buffer_.begin() );

            serialif_->dispatchRecvSize(
              recv_buffer_,
              ( 7 - data_buffer_.size() ),
              std::bind(
                &GyroJg35fd::callback_receive_data,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                2
              )
            );
          }
          else if( *( std::next( itr ) ) == 0x83 )
          {
            std::vector<unsigned char> tempv;
            tempv.clear();
            for( ; itr != data_buffer_.end(); ++itr )
            {
              tempv.push_back( *itr );
            }
            data_buffer_.clear();
            std::copy( tempv.begin(), tempv.end(), data_buffer_.begin() );

            serialif_->dispatchRecvSize(
              recv_buffer_,
              ( 9 - data_buffer_.size() ),
              std::bind(
                &GyroJg35fd::callback_receive_data,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                4
              )
            );
          }
          else{;}

          break;
        }
      }

    }

  }


  void GyroJg35fd::updateData( void )
  {
    uint16_t angle = 0;
    int16_t rate = 0;

    if( data_buffer_[1] == 0x81 )
    {
      uint8_t checksum = static_cast<uint8_t>( data_buffer_[1] + data_buffer_[2] + data_buffer_[3] + data_buffer_[4]);
      if( checksum == data_buffer_[5] )
      {
        angle = static_cast<uint16_t>( data_buffer_[3] << 8 ) + data_buffer_[4];

        std::lock_guard<std::mutex> lock(mtx_);
        yaw_angle_ = static_cast<double>( angle ) / static_cast<double>( 0xffff ) * M_PI * 2;
      }
    }
    else if( data_buffer_[1] == 0x82)
    {
      uint8_t checksum = static_cast<uint8_t>( data_buffer_[1] + data_buffer_[2] + data_buffer_[3] + data_buffer_[4]);
      if( checksum == data_buffer_[5] )
      {
        rate = static_cast<int16_t>( data_buffer_[3] << 8 ) + data_buffer_[4];

        std::lock_guard<std::mutex> lock(mtx_);
        yaw_rate_ = static_cast<double>( rate ) / static_cast<double>( 0x7fff ) * unitcon::angle::deg2rad( 200.0 );
      }
    }
    else if( data_buffer_[1] == 0x83)
    {
      uint8_t checksum = static_cast<uint8_t>( data_buffer_[1] + data_buffer_[2] + data_buffer_[3] + data_buffer_[4] + data_buffer_[5] + data_buffer_[6]);
      if( checksum == data_buffer_[7] )
      {
        angle = static_cast<uint16_t>( data_buffer_[3] << 8 ) + data_buffer_[4];
        rate = static_cast<int16_t>( data_buffer_[5] << 8 ) + data_buffer_[6];

        std::lock_guard<std::mutex> lock(mtx_);
        yaw_angle_ = static_cast<double>( angle ) / static_cast<double>( 0xffff ) * M_PI * 2.0 ;
        yaw_rate_ = static_cast<double>( rate ) / static_cast<double>( 0x7fff ) * unitcon::angle::deg2rad( 200.0 );
      }
    }
  
    data_buffer_.clear();

    std::cout << "Yaw angle is " << std::fixed << std::setprecision(6) << yaw_angle_ << " rad" << std::endl;
    std::cout << "Yaw rate is " << std::fixed << std::setprecision(6) << yaw_rate_ << " rad/s" << std::endl;

    serialif_->dispatchRecvUntil(
      recv_buffer_,
      REGEX_CONDITION_HEADER,
      std::bind(
        &GyroJg35fd::callback_receive_header,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );
  }


  void GyroJg35fd::configureOutput( uint8_t interval, uint8_t mode )
  {
    uint8_t checksum = static_cast<uint8_t>( mode + interval );

    auto send_buffer_ = std::make_shared<std::vector<unsigned char>>();
    send_buffer_->push_back( 0x02 );
    send_buffer_->push_back( mode );
    send_buffer_->push_back( interval );
    send_buffer_->push_back( checksum );
    send_buffer_->push_back( 0x0d );

    serialif_->dispatchSend(
      send_buffer_,
      std::bind(
        &GyroJg35fd::callback_sendSerial,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );    
  }


  void GyroJg35fd::requestConfigureBiasDrift( uint8_t request )
  {
    uint8_t checksum = static_cast<uint8_t>( 0x84 + request );

    auto send_buffer_ = std::make_shared<std::vector<unsigned char>>();
    send_buffer_->push_back( 0x02 );
    send_buffer_->push_back( 0x84 );
    send_buffer_->push_back( request );
    send_buffer_->push_back( checksum );
    send_buffer_->push_back( 0x0d );

    serialif_->dispatchSend(
      send_buffer_,
      std::bind(
        &GyroJg35fd::callback_sendSerial,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );
  }


  void GyroJg35fd::configureCalculate( uint8_t request )
  {
    uint8_t checksum = static_cast<uint8_t>( 0x86 + request );

    auto send_buffer_ = std::make_shared<std::vector<unsigned char>>();
    send_buffer_->push_back( 0x02 );
    send_buffer_->push_back( 0x86 );
    send_buffer_->push_back( request );
    send_buffer_->push_back( checksum );
    send_buffer_->push_back( 0x0d );

    serialif_->dispatchSend(
      send_buffer_,
      std::bind(
        &GyroJg35fd::callback_sendSerial,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );
  }

  void GyroJg35fd::requestResetAngle( double new_angle )
  {
    std::stringstream ss;
    ss <<
    std::setw(4) <<
    std::setfill('0') <<
    std::hex <<
    static_cast<int16_t>( new_angle / 180.0 * 32767.0 );
    
    uint8_t angle_letter[4] =
    {
      static_cast<uint8_t>( ss.str().c_str()[0] ),
      static_cast<uint8_t>( ss.str().c_str()[1] ),
      static_cast<uint8_t>( ss.str().c_str()[2] ),
      static_cast<uint8_t>( ss.str().c_str()[3] )
    };

    uint8_t checksum = static_cast<uint8_t>( 0x85 + angle_letter[0] +angle_letter[1] + angle_letter[2] + angle_letter[3]);

    auto send_buffer_ = std::make_shared<std::vector<unsigned char>>();
    send_buffer_->push_back( 0x02 );
    send_buffer_->push_back( 0x85 );
    send_buffer_->push_back( angle_letter[0] );
    send_buffer_->push_back( angle_letter[1] );
    send_buffer_->push_back( angle_letter[2] );
    send_buffer_->push_back( angle_letter[3] );
    send_buffer_->push_back( checksum );
    send_buffer_->push_back( 0x0d );

    serialif_->dispatchSend(
      send_buffer_,
      std::bind(
        &GyroJg35fd::callback_sendSerial,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );
  }


  void GyroJg35fd::configureAnalog( uint8_t mode, uint8_t range )
  {
    uint8_t checksum = static_cast<uint8_t>( 0xb3 + mode + range );

    auto send_buffer_ = std::make_shared<std::vector<unsigned char>>();
    send_buffer_->push_back( 0x02 );
    send_buffer_->push_back( 0xb3 );
    send_buffer_->push_back( mode );
    send_buffer_->push_back( range );
    send_buffer_->push_back( checksum );
    send_buffer_->push_back( 0x0d );

    serialif_->dispatchSend(
      send_buffer_,
      std::bind(
        &GyroJg35fd::callback_sendSerial,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );
  }


  void GyroJg35fd::callback_srv_control_output(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdControlOutput_Request> req,
    const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdControlOutput_Response> res
  )
  {
    using namespace japan_aeronautical_electronics_msgs::srv;

    res->result = Jg35fdControlOutput_Response::RESULT_DONE;

    if(
      req->mode != Jg35fdControlOutput_Request::MODE_STOP &&
      req->mode != Jg35fdControlOutput_Request::MODE_ONE_TIME &&
      req->mode != Jg35fdControlOutput_Request::MODE_CONTINUE_20MS &&
      req->mode != Jg35fdControlOutput_Request::MODE_CONTINUE_50MS &&
      req->mode != Jg35fdControlOutput_Request::MODE_CONTINUE_100MS &&
      req->mode != Jg35fdControlOutput_Request::MODE_CONTINUE_200MS &&
      req->mode != Jg35fdControlOutput_Request::MODE_CONTINUE_250MS &&
      req->mode != Jg35fdControlOutput_Request::MODE_CONTINUE_500MS &&
      req->mode != Jg35fdControlOutput_Request::MODE_CONTINUE_1000MS
    )
    { res->result = Jg35fdControlOutput_Response::RESULT_FAIL; }

    switch(
      req -> output != Jg35fdControlOutput_Request::OUTPUT_YAW &&
      req -> output != Jg35fdControlOutput_Request::OUTPUT_YAWRATE &&
      req -> output != Jg35fdControlOutput_Request::OUTPUT_BOTH
    )
    { res->result = Jg35fdControlOutput_Response::RESULT_FAIL; }

    if( res->result == Jg35fdControlOutput_Response::RESULT_DONE )
    { configureOutput( req->mode , req->output ); }
  }


  void GyroJg35fd::callback_srv_calibrate_bias_drift(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdCalibrateBiasDrift_Request> req,
    const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdCalibrateBiasDrift_Response> res
  )
  {
    using namespace japan_aeronautical_electronics_msgs::srv;

    res->result = Jg35fdCalibrateBiasDrift_Response::RESULT_DONE;

    if(
      req->request != Jg35fdCalibrateBiasDrift_Request::REQUEST_START &&
      req->request != Jg35fdCalibrateBiasDrift_Request::REQUEST_ABORT
    )
    { res->result = Jg35fdCalibrateBiasDrift_Response::RESULT_FAIL; }

    if( res->result == Jg35fdCalibrateBiasDrift_Response::RESULT_DONE )
    { requestConfigureBiasDrift( req->request ); }
  }


  void GyroJg35fd::callback_srv_control_calculate(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdControlCalculate_Request> req,
    const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdControlCalculate_Response> res
  )
  {
    using namespace japan_aeronautical_electronics_msgs::srv;

    res->result = Jg35fdControlCalculate_Response::RESULT_DONE;

    if(
      req->calculation != Jg35fdControlCalculate_Request::CALCULATION_DISABLE &&
      req->calculation != Jg35fdControlCalculate_Request::CALCULATION_ENABLE
    )
    { res->result = Jg35fdControlCalculate_Response::RESULT_FAIL; }

    if( res->result == Jg35fdControlCalculate_Response::RESULT_DONE )
    { configureCalculate( req->calculation ); }
  }


  void GyroJg35fd::callback_srv_reset_angle(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdResetAngle_Request> req,
    const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdResetAngle_Response> res
  )
  {
    using namespace japan_aeronautical_electronics_msgs::srv;

    res->result = Jg35fdResetAngle_Response::RESULT_DONE;

    if(
      req->yaw_angle_rad > 180.0 ||
      req->yaw_angle_rad < -180.0
    )
    { res->result = Jg35fdResetAngle_Response::RESULT_FAIL; }

    if( res->result == Jg35fdResetAngle_Response::RESULT_DONE )
    { requestResetAngle( req->yaw_angle_rad ); }
  }


  void GyroJg35fd::callback_srv_set_analog_mode(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdSetAnalogMode_Request> req,
    const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdSetAnalogMode_Response> res
  )
  {
    using namespace japan_aeronautical_electronics_msgs::srv;

    res->result = Jg35fdSetAnalogMode_Response::RESULT_DONE;

    if( req->analog_mode == Jg35fdSetAnalogMode_Request::ANALOG_MODE_YAW )
    {
      if(
        req->yaw_range != Jg35fdSetAnalogMode_Request::YAW_RANGE_10DEG &&
        req->yaw_range != Jg35fdSetAnalogMode_Request::YAW_RANGE_20DEG &&
        req->yaw_range != Jg35fdSetAnalogMode_Request::YAW_RANGE_45DEG &&
        req->yaw_range != Jg35fdSetAnalogMode_Request::YAW_RANGE_90DEG &&
        req->yaw_range != Jg35fdSetAnalogMode_Request::YAW_RANGE_180DEG
      )
      { res->result = Jg35fdSetAnalogMode_Response::RESULT_FAIL; }

      if( res->result == Jg35fdControlCalculate_Response::RESULT_DONE )
      { configureAnalog( req->analog_mode, req->yaw_range ); }
    }
    else if( req->analog_mode == Jg35fdSetAnalogMode_Request::ANALOG_MODE_YAWRATE )
    {
      if(
        req->yawrate_range != Jg35fdSetAnalogMode_Request::YAWRATE_RANGE_10DEG &&
        req->yawrate_range != Jg35fdSetAnalogMode_Request::YAWRATE_RANGE_20DEG &&
        req->yawrate_range != Jg35fdSetAnalogMode_Request::YAWRATE_RANGE_50DEG &&
        req->yawrate_range != Jg35fdSetAnalogMode_Request::YAWRATE_RANGE_100DEG &&
        req->yawrate_range != Jg35fdSetAnalogMode_Request::YAWRATE_RANGE_200DEG
      )
      { res->result = Jg35fdSetAnalogMode_Response::RESULT_FAIL; }

      if( res->result == Jg35fdControlCalculate_Response::RESULT_DONE )
      { configureAnalog( req->analog_mode, req->yawrate_range ); }
    }
    else
    { res->result = Jg35fdSetAnalogMode_Response::RESULT_FAIL; }

  }


}