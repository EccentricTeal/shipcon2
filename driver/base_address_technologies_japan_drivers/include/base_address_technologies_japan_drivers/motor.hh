#ifndef SHIPCON2DRIVER__BASE_ADDRESS_TECHNOLOGIES_JAPAN__MOTOR_TYPE0
#define SHIPCON2DRIVER__BASE_ADDRESS_TECHNOLOGIES_JAPAN__MOTOR_TYPE0

#include <rclcpp/rclcpp.hpp>
#include "hardware_communication_lib/udpcom.hh"

#include "shipcon_msgs/msg/motor_info.hpp"

#include <memory>
#include <exception>
#include <thread>
#include <mutex>
#include <string>
#include <tuple>

namespace shipcon::device::base_address_technologies_japan
{
  class CppMotor : public rclcpp::Node
  {
    /** Constants **/
    private:
      const uint16_t DATASIZE = 10; //Byte

    /** Member Objects **/
    private:
      std::string device_ip_;
      uint16_t my_port_, device_port_;
      std::unique_ptr<hwcomlib::UdpSend> udp_send_;
      std::unique_ptr<hwcomlib::UdpRecv> udp_recv_;
      std::unique_ptr<std::thread> threadptr_update_info_;
      std::mutex mtx_;
      shipcon_msgs::msg::MotorInfo motor_info_;
      
    /** Constructor, Destructor **/
    public:
      CppMotor( std::string node_name, std::string name_space );
      ~CppMotor();

    /** Methods **/
    public:
      void run( void );
    private:
      void getNetworkParam( void );

    /** Thread **/
    private:
      void thread_receiveUdp( void );
  };
}

#endif