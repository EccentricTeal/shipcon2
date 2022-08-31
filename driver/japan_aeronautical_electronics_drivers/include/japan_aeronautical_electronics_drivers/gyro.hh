#ifndef SHIPCON2DRIVER__JAPAN_AERONAUTICAL_ELECTRONICS__GYRO__HH
#define SHIPCON2DRIVER__JAPAN_AERONAUTICAL_ELECTRONICS__GYRO__HH

//ROS Packages
#include <rclcpp/rclcpp.hpp>
#include "hardware_communication_lib/serialcom.hh"
#include "unit_convert_lib/angle.hh"

//ROS Service
#include "japan_aeronautical_electronics_msgs/srv/jg35fd_control_output.hpp"
#include "japan_aeronautical_electronics_msgs/srv/jg35fd_calibrate_bias_drift.hpp"
#include "japan_aeronautical_electronics_msgs/srv/jg35fd_control_calculate.hpp"
#include "japan_aeronautical_electronics_msgs/srv/jg35fd_reset_angle.hpp"
#include "japan_aeronautical_electronics_msgs/srv/jg35fd_set_analog_range.hpp"
#include "japan_aeronautical_electronics_msgs/srv/jg35fd_set_analog_yawrate_range.hpp"

//STL
#include <memory>
#include <string>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <vector>
#include <cmath>
#include <utility>
#include <boost/asio.hpp>
#include <boost/regex.hpp>

namespace shipcon::device::japan_aeronautical_electronics
{
  class GyroJg35fd : public rclcpp::Node
  {
    /* Constants */
    private:
      const int BAUDRATE = 9600;
      enum TxInterval{
        once = 0x30,
        _20ms = 0x32,
        _50ms = 0x33,
        _100ms = 0x34,
        _200ms = 0x35,
        _250ms = 0x36,
        _500ms = 0x37,
        _1000ms = 0x38,
        stop = 0x39
      };
      enum OutputMode{
        yaw_angle = 0x81,
        yaw_rate = 0x82,
        both = 0x83
      };
      enum class AnalogAngleRange{
        current = 0x30,
        _10deg = 0x31,
        _20deg = 0x32,
        _45deg = 0x33,
        _90deg = 0x34,
        _180deg = 0x35
      };
      enum class AnalogYawrateRange{
        current = 0x30,
        _10deg = 0x31,
        _20deg = 0x32,
        _50deg = 0x33,
        _90deg = 0x34,
        _200deg = 0x35
      };
      boost::regex REGEX_CONDITION_HEADER = boost::regex("\x02[\x81-\x84]");
      const std::string DEVELOPPER_NAME = "JAPAN AERONAUTICAL ELECTRONICS CO., LTD.";
      const std::string DEVICE_TYPE = "YAW GYRO JG35FD";

    /* Private Member Objects*/
    private:
      //Serial Communication
      std::unique_ptr<hwcomlib::SerialCom> serialif_;
      std::string serial_port_name_;
      
      //ROS Service
      rclcpp::Service<japan_aeronautical_electronics_msgs::srv::Jg35fdControlOutput>::SharedPtr srv_control_output_;
      rclcpp::Service<japan_aeronautical_electronics_msgs::srv::Jg35fdCalibrateBiasDrift>::SharedPtr srv_calibrate_bias_drift_;
      rclcpp::Service<japan_aeronautical_electronics_msgs::srv::Jg35fdControlCalculate>::SharedPtr srv_control_calculate_;
      rclcpp::Service<japan_aeronautical_electronics_msgs::srv::Jg35fdResetAngle>::SharedPtr srv_reset_angle_;
      rclcpp::Service<japan_aeronautical_electronics_msgs::srv::Jg35fdSetAnalogRange>::SharedPtr srv_set_analog_range_;
      rclcpp::Service<japan_aeronautical_electronics_msgs::srv::Jg35fdSetAnalogYawrateRange>::SharedPtr srv_set_analog_yawrate_range_;
      std::string srvname_control_output_;
      std::string srvname_calibrate_bias_drift_;
      std::string srvname_control_calculate_;
      std::string srvname_reset_angle_;
      std::string srvname_set_analog_range_;
      std::string srvname_set_analog_yawrate_range_;
      
      //Buffers
      boost::asio::streambuf recv_buffer_;
      std::vector<unsigned char> data_buffer_;
      //Data
      double yaw_angle_;
      double yaw_rate_;
      //Utility
      std::mutex mtx_;

    /* Constructor, Destructor */
    public:
      GyroJg35fd( std::string node_name, std::string name_space );
      ~GyroJg35fd();

    /* Public Methods */
    public:
      void run( void );

    /* Private Methods */
    private:
      //ROS Service
      void initServiceParameter( void );
      void initService( void );

      //Serial Communication
      void initSerialParameter( void );
      bool initSerial( void );
      bool startSerial( void );
      void callback_sendSerial( const boost::system::error_code& ec, std::size_t sendsize );
      void callback_receive_header( const boost::system::error_code& ec, std::size_t recvsize );
      void callback_receive_data( const boost::system::error_code& ec, std::size_t recvsize, unsigned int datasize );
      void updateData( void );

      //Gyro Applications
      void configureOutput( TxInterval interval, OutputMode mode );
      void resetAngle( double new_angle );
      bool configureInternalCalculator( bool isEnable );
      AnalogAngleRange setAnalogAngleRange( AnalogAngleRange target );
      AnalogYawrateRange setAnalogYawrateRange( AnalogYawrateRange target );

      //Callback
      void callback_srv_control_output(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdControlOutput_Request> req,
        const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdControlOutput_Response> res
      );
      void callback_srv_calibrate_bias_drift(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdCalibrateBiasDrift_Request> req,
        const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdCalibrateBiasDrift_Response> res
      );
      void callback_srv_control_calculate(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdControlCalculate_Request> req,
        const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdControlCalculate_Response> res
      );
      void callback_srv_reset_angle(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdResetAngle_Request> req,
        const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdResetAngle_Response> res
      );
      void callback_srv_set_analog_range(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdSetAnalogRange_Request> req,
        const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdSetAnalogRange_Response> res
      );
      void callback_srv_set_analog_yawrate_range(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdSetAnalogYawrateRange_Request> req,
        const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdSetAnalogYawrateRange_Response> res
      );
  };
}

#endif