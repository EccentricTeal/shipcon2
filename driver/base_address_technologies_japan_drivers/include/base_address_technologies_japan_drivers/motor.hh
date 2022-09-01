#ifndef SHIPCON2DRIVER__BASE_ADDRESS_TECHNOLOGIES_JAPAN__MOTOR__HH
#define SHIPCON2DRIVER__BASE_ADDRESS_TECHNOLOGIES_JAPAN__MOTOR__HH

#include <rclcpp/rclcpp.hpp>
#include "hardware_communication_lib/udpcom.hh"
#include "shipcon_msgs/srv/get_motor_spec.hpp"

#include "shipcon_msgs/msg/motor_info.hpp"
#include "shipcon_msgs/msg/motor_command.hpp"
#include "shipcon_msgs/msg/propeller_info.hpp"
#include "shipcon_msgs/msg/propeller_command.hpp"
#include "shipcon_msgs/srv/get_motor_spec.hpp"
#include "unit_convert_lib/angle.hh"

#include <memory>
#include <exception>
#include <thread>
#include <mutex>
#include <string>
#include <tuple>

using namespace std::literals::chrono_literals;


namespace shipcon::device::base_address_technologies_japan
{
  class CppMotor : public rclcpp::Node
  {
    /** Constants **/
    private:
      const uint16_t DATASIZE = 10; //Byte
      const double MOTOR_MAX_RPM = 3000.0;
      const double PROP_MAX_ANGLE_AHEAD_DEG = 30.0;
      const double PROP_MAX_ANGLE_ASTERN_DEG = 20.0;
      const std::string DEVELOPPER_NAME = "BASE ADDRESS TECHNOLOGIES JAPAN";
      const std::string DEVICE_TYPE = "CPP Motor Driver";
      const std::chrono::milliseconds pubinfo_rate_ = 50ms;
      const std::chrono::milliseconds sendctrl_rate_ = 100ms;

    /** Member Objects **/
    private:
      std::string device_ip_, device_ip_mask_;
      uint16_t my_port_, device_port_;
      std::unique_ptr<hwcomlib::UdpSend> udp_send_;
      std::unique_ptr<hwcomlib::UdpRecv> udp_recv_;

      std::unique_ptr<std::thread> threadptr_pub_info_;
      rclcpp::TimerBase::SharedPtr timerptr_sendcontrol_timer_;
      std::mutex mtx_;

      shipcon_msgs::msg::PropellerCommand prop_command_;
      shipcon_msgs::msg::MotorCommand motor_command_;
      shipcon_msgs::msg::PropellerInfo prop_info_;
      shipcon_msgs::msg::MotorInfo motor_info_;
      rclcpp::Service<shipcon_msgs::srv::GetMotorSpec>::SharedPtr srv_get_motor_spec_;
      rclcpp::Publisher<shipcon_msgs::msg::MotorInfo>::SharedPtr pub_motor_info_;
      rclcpp::Publisher<shipcon_msgs::msg::PropellerInfo>::SharedPtr pub_prop_info_;
      rclcpp::Subscription<shipcon_msgs::msg::MotorCommand>::SharedPtr sub_motor_control_;
      rclcpp::Subscription<shipcon_msgs::msg::PropellerCommand>::SharedPtr sub_prop_control_;
      std::string subname_motor_control_;
      std::string subname_prop_control_;
      
    /** Constructor, Destructor **/
    public:
      CppMotor( std::string node_name, std::string name_space );
      ~CppMotor();

    /** Methods **/
    public:
      void run( void );
    private:
      void initParameterHandler( void );
      void initUdp( void );
      void initPublisher( void );
      void initSubscription( void );
      void initService( void );
      void getNetworkParam( void );
      void getTopicParam( void );
      void updateInfo( std::vector<char>& buffer );
      std::string formSendData( void );
      void evaluateInfo( void );

    /** Thread **/
    private:
      void thread_sendControl( void );
      void thread_receiveUdp( void );

    /** Callback **/
    private:
      void callbackMotorCommand_sub_( const shipcon_msgs::msg::MotorCommand::SharedPtr msg );
      void callbackPropellerCommand_sub_( const shipcon_msgs::msg::PropellerCommand::SharedPtr msg );
      void callback_srv_get_motor_spec(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<shipcon_msgs::srv::GetMotorSpec_Request> req,
        const std::shared_ptr<shipcon_msgs::srv::GetMotorSpec_Response> res
      );
  };


  class NormalMotor : public rclcpp::Node
  {
    /** Constants **/
    private:
      const uint16_t DATASIZE = 10; //Byte
      const double MOTOR_MAX_RPM = 3000.0;
      const std::string DEVELOPPER_NAME = "BASE ADDRESS TECHNOLOGIES JAPAN";
      const std::string DEVICE_TYPE = "Normal Motor Driver";
      const std::chrono::milliseconds sendctrl_rate_ = 50ms;

    /** Member Objects **/
    private:
      std::string device_ip_, device_ip_mask_;
      uint16_t my_port_, device_port_;
      std::unique_ptr<hwcomlib::UdpSend> udp_send_;
      std::unique_ptr<hwcomlib::UdpRecv> udp_recv_;

      std::unique_ptr<std::thread> threadptr_pub_info_;
      rclcpp::TimerBase::SharedPtr timerptr_sendcontrol_timer_;
      std::mutex mtx_;

      shipcon_msgs::msg::MotorCommand motor_command_;
      shipcon_msgs::msg::MotorInfo motor_info_;
      rclcpp::Service<shipcon_msgs::srv::GetMotorSpec>::SharedPtr srv_get_motor_spec_;
      rclcpp::Publisher<shipcon_msgs::msg::MotorInfo>::SharedPtr pub_motor_info_;
      rclcpp::Subscription<shipcon_msgs::msg::MotorCommand>::SharedPtr sub_motor_control_;
      std::string subname_motor_control_;
      
    /** Constructor, Destructor **/
    public:
      NormalMotor( std::string node_name, std::string name_space );
      ~NormalMotor();

    /** Methods **/
    public:
      void run( void );
    private:
      void initParameterHandler( void );
      void initUdp( void );
      void initPublisher( void );
      void initSubscription( void );
      void initService( void );
      void getNetworkParam( void );
      void getTopicParam( void );
      void updateInfo( std::vector<char>& buffer );
      std::string formSendData( void );
      void evaluateInfo( void );

    /** Thread **/
    private:
      void thread_sendControl( void );
      void thread_receiveUdp( void );

    /** Callback **/
    private:
      void callbackMotorCommand_sub_( const shipcon_msgs::msg::MotorCommand::SharedPtr msg );
      void callback_srv_get_motor_spec(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<shipcon_msgs::srv::GetMotorSpec_Request> req,
        const std::shared_ptr<shipcon_msgs::srv::GetMotorSpec_Response> res
      );
  };
}

#endif