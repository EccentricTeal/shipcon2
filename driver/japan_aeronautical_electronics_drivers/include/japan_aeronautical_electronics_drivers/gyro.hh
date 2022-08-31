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
#include "japan_aeronautical_electronics_msgs/srv/jg35fd_set_analog_mode.hpp"

//STL
#include <memory>
#include <string>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <vector>
#include <cmath>
#include <utility>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/regex.hpp>

namespace shipcon::device::japan_aeronautical_electronics
{
  class GyroJg35fd : public rclcpp::Node
  {
    /* Constants */
    private:
      const int BAUDRATE = 9600;
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
      rclcpp::Service<japan_aeronautical_electronics_msgs::srv::Jg35fdSetAnalogMode>::SharedPtr srv_set_analog_mode_;
      std::string srvname_control_output_;
      std::string srvname_calibrate_bias_drift_;
      std::string srvname_control_calculate_;
      std::string srvname_reset_angle_;
      std::string srvname_set_analog_mode_;
      
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
      void configureOutput( uint8_t interval, uint8_t mode );
      void requestConfigureBiasDrift( uint8_t request );
      void configureCalculate( uint8_t request );
      void requestResetAngle( double new_angle );
      void configureAnalog( uint8_t mode, uint8_t range );

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
      void callback_srv_set_analog_mode(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdSetAnalogMode_Request> req,
        const std::shared_ptr<japan_aeronautical_electronics_msgs::srv::Jg35fdSetAnalogMode_Response> res
      );
  };
}

#endif