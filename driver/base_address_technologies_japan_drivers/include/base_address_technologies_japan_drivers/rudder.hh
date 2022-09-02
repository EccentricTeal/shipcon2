#ifndef SHIPCON2DRIVER__BASE_ADDRESS_TECHNOLOGIES_JAPAN__RUDDER__HH
#define SHIPCON2DRIVER__BASE_ADDRESS_TECHNOLOGIES_JAPAN__RUDDER__HH

#include <rclcpp/rclcpp.hpp>
#include "hardware_communication_lib/udpcom.hh"
#include "shipcon_msgs/srv/get_rudder_spec.hpp"

#include "shipcon_msgs/msg/rudder_info_twin.hpp"
#include "shipcon_msgs/msg/rudder_command_twin.hpp"
#include "shipcon_msgs/srv/get_rudder_spec.hpp"
#include "unit_convert_lib/angle.hh"

#include <memory>
#include <exception>
#include <thread>
#include <mutex>
#include <string>
#include <tuple>


namespace shipcon::device::base_address_technologies_japan
{
  using namespace std::literals::chrono_literals;

  class VectwinRudder : public rclcpp::Node
  { 
    /** Constants **/
    private:
      const uint16_t DATASIZE = 10; //Byte
      const double RUDDER_MAX_ANGLE_PORT_DEG = 30.0;
      const double RUDDER_MAX_ANGLE_STBD_DEG = 30.0;
      const std::string DEVELOPPER_NAME = "BASE ADDRESS TECHNOLOGIES JAPAN";
      const std::string DEVICE_TYPE = "Vectwin Rudder Driver";
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

      shipcon_msgs::msg::RudderCommandTwin rudder_command_;
      shipcon_msgs::msg::RudderInfoTwin rudder_info_;
      rclcpp::Service<shipcon_msgs::srv::GetRudderSpec>::SharedPtr srv_get_rudder_spec_;
      rclcpp::Publisher<shipcon_msgs::msg::RudderInfoTwin>::SharedPtr pub_rudder_info_;
      rclcpp::Subscription<shipcon_msgs::msg::RudderCommandTwin>::SharedPtr sub_rudder_control_;
      std::string subname_rudder_control_;
      
    /** Constructor, Destructor **/
    public:
      VectwinRudder( std::string node_name, std::string name_space );
      ~VectwinRudder();

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
      void callbackRudderCommand_sub( const shipcon_msgs::msg::RudderCommandTwin::SharedPtr msg );
      void callback_srv_get_rudder_spec(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<shipcon_msgs::srv::GetRudderSpec_Request> req,
        const std::shared_ptr<shipcon_msgs::srv::GetRudderSpec_Response> res
      );
  };

}

#endif