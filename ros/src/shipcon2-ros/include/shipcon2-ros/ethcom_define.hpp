#ifndef ETHCOM_DEFINE_HPP
#define ETHCOM_DEFINE_HPP

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


EthcomSocketClass::EthcomSocketClass()
{
    ;
}

EthcomSocketClass::~EthcomSocketClass()
{
    ;
}

EthcomSocketClass::setIpAddr( std::string ip_addr )
{
    ;
}