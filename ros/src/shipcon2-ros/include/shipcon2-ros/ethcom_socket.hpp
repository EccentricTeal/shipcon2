#ifndef ETHCOM_SOCKET_HPP
#define ETHCOM_SOCKET_HPP

#include <string>
#include <netinet/in.h>

class EthcomSocketClass
{
public:
    EthcomSocketClass();
    ~EthcomSocketClass();

    //API
    int setIpAddr( std::string ip_addr );
    std::string getIpAddr( void );

private:
    sockaddr_in ip_;
    

};

#endif