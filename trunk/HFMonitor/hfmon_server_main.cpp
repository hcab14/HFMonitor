// -*- C++ -*-
// $Id$
#include <iostream>
#include <iterator>
#include <vector>
#include <set>
#include <string>
#include <deque>

#include "perseus++.h"
#include "protocol.hpp"

#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "run.h"

class data_connection : private boost::noncopyable {
public:
  typedef boost::shared_ptr<boost::asio::ip::tcp::socket> tcp_socket_ptr;

  typedef std::deque<std::vector<char> > ListOfPackets;

  data_connection(boost::asio::io_service& io_service,
		  tcp_socket_ptr p)
    : io_service_(io_service)
    , strand_(io_service)
    , tcp_socket_ptr_(p)
    , isOpen_(true) {}

  ~data_connection() {
    close();
  }

  void close() {  tcp_socket_ptr_->close(); isOpen_= false; }
  bool is_open() const { return isOpen_; }

  void send(boost::array<boost::asio::const_buffer, 2> bufs) {
    if (is_open()) {
      try {
	tcp_socket_ptr_->send(bufs);
      } catch (const boost::system::system_error& e) {
	std::cout << "send error:_ " << e.what() << std::endl;
	close();
      }
    }
  }
  
protected:
  
private:
  boost::asio::io_service& io_service_;
  boost::asio::strand      strand_;
  tcp_socket_ptr           tcp_socket_ptr_;
  bool                     isOpen_;
  ListOfPackets            listOfPackets_;
} ;

class server : private boost::noncopyable {
public:
  typedef boost::shared_ptr<boost::asio::ip::tcp::socket> tcp_socket_ptr;
  typedef std::set<tcp_socket_ptr> Sockets;
  typedef boost::shared_ptr<data_connection> data_connection_ptr;
  typedef std::set<data_connection_ptr> data_connections;

  server(boost::asio::io_service& io_service,
	 const boost::asio::ip::tcp::endpoint& endpoint_ctrl,
	 const boost::asio::ip::tcp::endpoint& endpoint_data,
	 Perseus::ReceiverPtr recPtr,
	 unsigned usbBufferSize)
    : io_service_(io_service)
    , strand_(io_service)
    , acceptor_ctrl_(io_service, endpoint_ctrl)
    , acceptor_data_(io_service, endpoint_data)
    , recPtr_(recPtr)
    , usbBufferSize_(usbBufferSize)
    , sampleNumber_(0) {
    // control setup
    {
      acceptor_ctrl_.set_option(boost::asio::socket_base::reuse_address(true));
      acceptor_ctrl_.listen();
      tcp_socket_ptr new_socket(new boost::asio::ip::tcp::socket(acceptor_ctrl_.get_io_service()));
      acceptor_ctrl_.async_accept(*new_socket,
				  strand_.wrap(boost::bind(&server::handle_accept_ctrl, this,
							   boost::asio::placeholders::error, new_socket)));
    }
    // data setup
    {
      acceptor_data_.set_option(boost::asio::socket_base::reuse_address(true));
      acceptor_data_.listen();
      tcp_socket_ptr new_socket(new boost::asio::ip::tcp::socket(acceptor_data_.get_io_service()));
      acceptor_data_.async_accept(*new_socket,
				  strand_.wrap(boost::bind(&server::handle_accept_data, this,
							   boost::asio::placeholders::error, new_socket)));
    }
    //
    ptimeOfCallback_ = ptimeOfCallbackInterpolated_ = boost::posix_time::microsec_clock::universal_time();
    std::cout << "ptime_ = " << ptimeOfCallback_ << std::endl;
    recPtr_->startAsyncInput(usbBufferSize_, server::receiverCallback, this);
  }
  
  void handle_accept_ctrl(const boost::system::error_code& ec, tcp_socket_ptr socket) {
    std::cout << "servce::handle_accept_ctrl error_code= " << ec << std::endl;
    std::cout << "remote endpoint= " << socket->remote_endpoint() << std::endl;
    if (!ec) {
      ctrl_sockets_.insert(socket);
      std::cout << "socket remote_ep = " << socket->remote_endpoint() << " "
		<< (socket->is_open() ? "open" : "closed") << std::endl;
      
      tcp_socket_ptr new_socket(new boost::asio::ip::tcp::socket(acceptor_ctrl_.get_io_service()));      
      acceptor_ctrl_.async_accept(*new_socket,
				  strand_.wrap(boost::bind(&server::handle_accept_ctrl, this,
							   boost::asio::placeholders::error, new_socket)));
    }
  }
  void handle_accept_data(const boost::system::error_code& ec, tcp_socket_ptr socket) {
    std::cout << "servce::handle_accept_data error_code= " << ec << std::endl;
    if (!ec) {
      std::cout << "remote endpoint= " << socket->remote_endpoint() << std::endl;

      data_connections_.insert(data_connection_ptr(new data_connection(io_service_, socket)));

      tcp_socket_ptr new_socket(new boost::asio::ip::tcp::socket(acceptor_data_.get_io_service()));      
      acceptor_data_.async_accept(*new_socket,
				  strand_.wrap(boost::bind(&server::handle_accept_data, this,
							   boost::asio::placeholders::error, new_socket)));
    } else {
      // error 
    }
  }

  Header getHeader(const unsigned nSamples) {
    return Header((sampleNumber_+=nSamples) - nSamples,
		  recPtr_->sampleRate(),
		  recPtr_->ddcCenterFrequency(),
		  nSamples,
		  0, // TODO
		  recPtr_->attenId(),
		  recPtr_->enablePresel(),
		  recPtr_->enablePreamp(),
		  recPtr_->enableDither());
  }

  static int receiverCallback(void *buf, int buf_size, void *extra) {
    server* sp= (server* )extra;
    const unsigned nSamples   = buf_size/6;
    const Header   header(sp->getHeader(nSamples));

    const double dt_microseconds(1e6 * double(nSamples) / sp->recPtr_->sampleRate());
    const boost::posix_time::ptime now(boost::posix_time::microsec_clock::universal_time());
    const boost::posix_time::time_duration dt(now-sp->ptimeOfCallback_);
    
    if (std::abs(dt.total_microseconds() - dt_microseconds) < dt_microseconds/100) {
      sp->ptimeOfCallbackInterpolated_ = now;
    } else {
      sp->ptimeOfCallbackInterpolated_ += dt;
    }
    sp->ptimeOfCallback_= now;

    std::cout << "receiverCallback ptCB,ptCBInt, dt_usec, dt_usec,ddt_usec= " 
	      << sp->ptimeOfCallback_ << " " << sp->ptimeOfCallbackInterpolated_ << " "
	      << dt_microseconds << " " << dt.total_microseconds() << " "
	      << std::abs(dt.total_microseconds() - dt_microseconds) << std::endl;;    

    const boost::array<boost::asio::const_buffer, 2> bufs = {
      boost::asio::const_buffer(&header, sizeof(Header)),
      boost::asio::const_buffer(buf, buf_size)
    };
    if (sp->sampleNumber_ % 85000 == 0)
      std::cout << "data_connections_.size*() = " << sp->data_connections_.size() 
		<< " sampleNumber=" << sp->sampleNumber_ << std::endl;
    sp->sendDataToClients(bufs);
    return 0;
  }

  void sendDataToClients(const boost::array<boost::asio::const_buffer, 2>& bufs) {
    for (data_connections::iterator i(data_connections_.begin()); i!=data_connections_.end();) {
      if ((*i)->is_open()) 
	(*i++)->send(bufs);      
      else
	data_connections_.erase(i++);
    }
  }

  void stop() {
    recPtr_->stopAsyncInput();
    io_service_.post(boost::bind(&server::do_stop, this));
  }

  void do_stop() {
    ctrl_sockets_.clear();
    data_connections_.clear();
    io_service_.stop();
  }

protected:
private:
  boost::asio::io_service&       io_service_;
  boost::asio::strand            strand_;
  boost::asio::ip::tcp::acceptor acceptor_ctrl_;
  boost::asio::ip::tcp::acceptor acceptor_data_;
  Sockets                        ctrl_sockets_;
  Perseus::ReceiverPtr           recPtr_;
  unsigned                       usbBufferSize_;
  boost::int64_t                 sampleNumber_;
  boost::posix_time::ptime        ptimeOfCallback_;
  boost::posix_time::ptime        ptimeOfCallbackInterpolated_;
  data_connections               data_connections_;

  boost::asio::streambuf request_;
  boost::asio::streambuf response_;
} ;


int main(int argc, char* argv[])
{
  try {
    std::string filename((argc > 1 ) ? argv[1] : "config.xml");
    boost::property_tree::ptree pt;
    read_xml(filename, pt);
    std::cout << "debug.filename=" << pt.get<std::string>("debug.filename") << std::endl;

    perseus_set_debug(3);

    Perseus p;
    const unsigned numPerseus(p.numPerseus());
    if (numPerseus == 0)
      throw std::runtime_error("numPerseus == 0");

    Perseus::ReceiverPtr pp(p.getReceiverPtr(0));

    pp->downloadFirmware();
    pp->fpgaConfig(pt.get<int>("perseus.fpga.samplerate"));		   

    // todo: get from config
    pp->setAttenuator(PERSEUS_ATT_0DB);

    // todo: get flags from config
    pp->setADC(true, false);

    // todo: get flag from config
    pp->setDdcCenterFreq(pt.get<double>("perseus.qrg_Hz"), 1);

    boost::asio::io_service io_service;
    server s(io_service,
	     boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 
					    pt.get<unsigned>("server.ctrl.port")),
	     boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 
					    pt.get<unsigned>("server.data.port")),
	     pp,
	     pt.get<unsigned>("perseus.USBBufferSize"));

    run(io_service, s);    
  } catch (std::exception &e) {
    std::cout << "Error: " << e.what() << "\n";
  }
  return 0;
}
