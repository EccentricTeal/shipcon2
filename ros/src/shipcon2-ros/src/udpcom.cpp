#include "shipcon2-ros/udpcom.hpp"




UdpSendClass::UdpSendClass( std::string dest_ip, uint16_t dest_port ):
iosrv_ptr_( new boost::asio::io_service )
{
    //Initializing Socket
    socket_ptr_ = std::make_unique<boost::asio::ip::udp::socket>( *iosrv_ptr_ );
    socket_ptr_->open( boost::asio::ip::udp::v4() );
    setDestEndpoint( dest_ip, dest_port );
}


UdpSendClass::~UdpSendClass()
{
    socket_ptr_->close();
}


void UdpSendClass::setDestEndpoint( std::string dest_ip, uint16_t dest_port )
{
    boost::asio::ip::address_v4 ip_addr;
    
    ip_addr = boost::asio::ip::address_v4::from_string( dest_ip );
    mtx_.lock();
    dest_endpoint_ = boost::asio::ip::udp::endpoint( ip_addr, dest_port );
    mtx_.unlock();
}


int32_t UdpSendClass::sendData( std::string send_data )
{
    mtx_.lock();
    boost::asio::ip::udp::endpoint destep = dest_endpoint_;
    mtx_.unlock();
    std::system_error error;

    const size_t sent = socket_ptr_->send_to( boost::asio::buffer(send_data), destep );
}





UdpRecvClass::UdpRecvClass( uint16_t udp_port ):
iosrv_ptr_( new boost::asio::io_service )
{
    //Initializing Socket
    socket_ptr_ = std::make_unique<boost::asio::ip::udp::socket>( *iosrv_ptr_ );
    socket_ptr_->open( boost::asio::ip::udp::v4() );
}


UdpRecvClass::~UdpRecvClass()
{
    socket_ptr_->close();
}


std::tuple<size_t, std::string> UdpRecvClass::recvData( std::string src_ip, std::string* buffer )
{
    boost::asio::ip::udp::endpoint src_endpoint;

    size_t sent = socket_ptr_->receive_from( boost::asio::buffer(recvbuf_), src_endpoint );

    return std::make_tuple( sent, src_endpoint.address().to_string() );
}
