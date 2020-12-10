#ifndef UDPCOM_HPP
#define UDPCOM_HPP

/**
 * @file udpcom.hpp
 * @brief Simple class library for UDP communication(Send or Receive) in C++
 * @author Suisei WADA
 * @date 20th November, 2020
 * 
 * @details
 * @note
 */

#include <string>
#include <memory>
#include <mutex>
#include <vector>
#include <array>
#include <tuple>
#include <system_error.hpp>
#include <boost/asio.hpp>


/**
 * Core class for UDP communication.
 */
class UdpSendClass
{       
    /* Public member functions */
    public:
    UdpSendClass( std::string dest_ip, uint16_t dest_port );
    ~UdpSendClass();
    void setDestEndpoint(  std::string dest_ip, uint16_t dest_port );
    int32_t sendData( std::string send_data );    

    /* Class member functions */
    private:

    /* Class member variables */
    private:
    std::mutex mtx_;
    boost::asio::ip::udp::endpoint dest_endpoint_;
    std::unique_ptr<boost::asio::io_service> iosrv_ptr_;
    std::unique_ptr<boost::asio::ip::udp::socket> socket_ptr_;
};


class UdpRecvClass
{      
    /* Public member functions */
    public:
    UdpRecvClass( uint16_t udp_port );
    ~UdpRecvClass();
    std::tuple<size_t, std::string> recvData( std::string src_ip );

    /* Class member functions */
    private:
    void init_( void );
    std::vector<char> recvdata_;

    /* Class member variables */
    private:
    std::mutex mtx_;
    std::unique_ptr<boost::asio::io_service> iosrv_ptr_;
    std::unique_ptr<boost::asio::ip::udp::socket> socket_ptr_;
};

#endif



