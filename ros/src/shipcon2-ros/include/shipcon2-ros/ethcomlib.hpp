#ifndef ETHCOMLIB_HPP
#define ETHCONLIB_HPP

/**
 * @file ethcomlib.hpp
 * @brief Simple class library for ehternet communication in C++
 * @author Suisei WADA
 * @date 4th November, 2020
 * 
 * @details
 * @note
 */

#include <string>

/**
 * Core class for ethernet communication as server
 */
class EthernetServer
{
public:
    EthernetServer(/* args */);
    ~EthernetServer();
    void setMyIpAddr ( std::string ip_addr);
        
private:
    int init( void );    
    int send_udp( int send_port, std::string dest_ip, std::string send_data );
    int recieve_udp( std::string src_ip, std::string* msg );
    


};

EthernetServer::EthernetServer(/* args */)
{
}

EthernetServer::~EthernetServer()
{
}



