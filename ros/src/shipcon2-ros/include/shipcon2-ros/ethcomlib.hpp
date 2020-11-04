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


/**
 * Core class for ethernet communication as server
 */
class EthernetServer
{
public:
    EthernetServer(/* args */);
    ~EthernetServer();
        
private:
    int init( void );
    void setMyIpAddr ()


};

EthernetServer::EthernetServer(/* args */)
{
}

EthernetServer::~EthernetServer()
{
}



