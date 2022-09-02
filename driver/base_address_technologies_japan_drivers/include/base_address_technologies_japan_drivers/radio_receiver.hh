#ifndef SHIPCON2DRIVER__BASE_ADDRESS_TECHNOLOGIES_JAPAN__RADIO_RECEIVER__HH
#define SHIPCON2DRIVER__BASE_ADDRESS_TECHNOLOGIES_JAPAN__RADIO_RECEIVER__HH

#include <rclcpp/rclcpp.hpp>
#include "hardware_communication_lib/udpcom.hh"

#include "base_address_technologies_japan_msgs/msg/radio_receiver_status.hpp"
#include "base_address_technologies_japan_msgs/msg/radio_receiver_ship_mode.hpp"
#include "base_address_technologies_japan_msgs/msg/radio_receiver_vectwin_mode.hpp"
#include "sensor_msgs/msg/joy.hpp"
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

  class VectwinRadioReceiver : public rclcpp::Node
  { 
    /** Constants **/
    private:
      const uint16_t DATASIZE = 10; //Byte
      const std::string DEVELOPPER_NAME = "BASE ADDRESS TECHNOLOGIES JAPAN";
      const std::string DEVICE_TYPE = "Vectwin Radio Receiver";
      const std::chrono::milliseconds pubinfo_rate_ = 50ms;

    /** Member Objects **/
    private:
      std::string device_ip_, device_ip_mask_;
      uint16_t my_port_, device_port_;
      std::unique_ptr<hwcomlib::UdpSend> udp_send_;
      std::unique_ptr<hwcomlib::UdpRecv> udp_recv_;

      std::unique_ptr<std::thread> threadptr_pub_info_;
      rclcpp::TimerBase::SharedPtr timerptr_sendcontrol_timer_;
      std::mutex mtx_;

      base_address_technologies_japan_msgs::msg::RadioReceiverStatus status_;
      base_address_technologies_japan_msgs::msg::RadioReceiverShipMode ship_mode_;
      base_address_technologies_japan_msgs::msg::RadioReceiverVectwinMode vectwin_mode_;
      sensor_msgs::msg::Joy joy_;

      rclcpp::Publisher<base_address_technologies_japan_msgs::msg::RadioReceiverStatus>::SharedPtr pub_status_;
      rclcpp::Publisher<base_address_technologies_japan_msgs::msg::RadioReceiverShipMode>::SharedPtr pub_ship_mode_;
      rclcpp::Publisher<base_address_technologies_japan_msgs::msg::RadioReceiverVectwinMode>::SharedPtr pub_vectwin_mode_;
      rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_joy_;
      
    /** Constructor, Destructor **/
    public:
      VectwinRadioReceiver( std::string node_name, std::string name_space );
      ~VectwinRadioReceiver();

    /** Methods **/
    public:
      void run( void );
    private:
      void initUdp( void );
      void initPublisher( void );
      void getNetworkParam( void );
      void updateInfo( std::vector<char>& buffer );
      void evaluateInfo( void );

    /** Thread **/
    private:
      void thread_receiveUdp( void );

    /** Callback **/
    private:
  };

}

#endif