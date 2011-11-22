// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _CLIENT_TCP_HPP_cm100729_
#define _CLIENT_TCP_HPP_cm100729_

#include <iostream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>

#include "protocol.hpp"
#include "logging.hpp"

template<typename PROCESSOR>
class ClientTCP : private boost::noncopyable {
private:
  enum {
    maxBufferSize = 1024*1024
  } ;
  typedef enum {
    ReceivedHeader,
    ReceivedData
  } ReceivedDataType;

public:
  ClientTCP(boost::asio::io_service& io_service,
            boost::asio::ip::tcp::resolver::iterator endpoint_iterator,
            const boost::property_tree::ptree& config)
    : p_(config)
    , service_(io_service)
    , strand_(io_service)
    , socket_(io_service)
    , header_()
    , tickBuffer_('a')
    , timer_(service_, boost::posix_time::seconds(1)) {
    boost::asio::ip::tcp::endpoint endPoint(*endpoint_iterator); 
    socket_.async_connect(endPoint, 
                          strand_.wrap(boost::bind(&ClientTCP::onConnect, 
                                                   this, 
                                                   boost::asio::placeholders::error, 
                                                   ++endpoint_iterator)));
  }
  
  void stop() { 
    service_.post(boost::bind(&ClientTCP::doClose, this)); 
  }

protected:
  void onConnect(const boost::system::error_code& errorCode, 
                 boost::asio::ip::tcp::resolver::iterator endpoint_iterator) { 
    if (errorCode == 0) { 
      async_receive_header();
      timer_.async_wait(strand_.wrap(boost::bind(&ClientTCP::onTick, this)));
    } else if (endpoint_iterator != boost::asio::ip::tcp::resolver::iterator()) { 
      socket_.close();
      boost::asio::ip::tcp::endpoint endPoint = *endpoint_iterator;
      socket_.async_connect(endPoint,
                            strand_.wrap(boost::bind(&ClientTCP::onConnect,
                                                     this,
                                                     boost::asio::placeholders::error,
                                                     ++endpoint_iterator)));
    }
  }

  void onTick() {
    // send tick to server
    boost::asio::async_write(socket_,
                             boost::asio::buffer(&tickBuffer_, sizeof(tickBuffer_)),
                             strand_.wrap(boost::bind(&ClientTCP::handle_write_tick,
                                                      this,
                                                      boost::asio::placeholders::error,
                                                      boost::asio::placeholders::bytes_transferred,
                                                      tickBuffer_)));
    timer_.expires_at(timer_.expires_at() + boost::posix_time::seconds(1));
    timer_.async_wait(strand_.wrap(boost::bind(&ClientTCP::onTick, this)));
    tickBuffer_ = (tickBuffer_ != 'z') ? tickBuffer_+1 : 'a';
  }
  void handle_write_tick(const boost::system::error_code& ec, std::size_t bytes_transferred, char tickValue) {
    if (ec || bytes_transferred == 0) {
      LOG_WARNING(str(boost::format("handle_write_tick aborting: error '%s' bytes_transferred=%d")
                      % ec.message()
                      % bytes_transferred));
      doClose();
      service_.stop();
    } else
      LOG_INFO(str(boost::format("tick '%c' sent") % tickValue));
  }

  void onReceive(const boost::system::error_code& errorCode, ReceivedDataType rdt) { 
    if (errorCode == 0) {
      switch (rdt) {
      case ReceivedHeader:
        async_receive_data();   break;
      case ReceivedData:
        p_.procRaw(header_,
                   std::vector<char>::const_iterator(dataBuffer_.begin()),
                   std::vector<char>::const_iterator(dataBuffer_.begin()+header_.numberOfSamples()*6));
        async_receive_header(); break;
      default:
        ; // do nothing 
      }
    } else doClose(); 
  }

  void async_receive_header() {
    boost::asio::async_read(socket_, boost::asio::buffer(&header_, sizeof(Header)),
                            strand_.wrap(boost::bind(&ClientTCP::onReceive,
                                                     this,
                                                     boost::asio::placeholders::error,
                                                     ReceivedHeader)));
  }
  void async_receive_data() {
    ASSERT_THROW(header_.numberOfSamples()*6 < maxBufferSize);
    boost::asio::async_read(socket_, boost::asio::buffer(&dataBuffer_, header_.numberOfSamples()*6),
                            strand_.wrap(boost::bind(&ClientTCP::onReceive,
                                                     this,
                                                     boost::asio::placeholders::error,
                                                     ReceivedData)));
  }

  void doClose() {
    socket_.close();
  }
  
private:
  PROCESSOR                         p_;
  boost::asio::io_service&          service_;
  boost::asio::strand               strand_;
  boost::asio::ip::tcp::socket      socket_;
  Header                            header_;
  boost::array<char, maxBufferSize> dataBuffer_;
  char                              tickBuffer_;
  boost::asio::deadline_timer       timer_;
} ;

#endif // _CLIENT_TCP_HPP_cm100729_
