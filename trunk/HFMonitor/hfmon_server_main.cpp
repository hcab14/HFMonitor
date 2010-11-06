// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <iterator>
#include <vector>
#include <set>
#include <string>
#include <deque>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/foreach.hpp>

#include "perseus++.h"
#include "Filter.hpp"
#include "protocol.hpp"
#include "run.hpp"

class data_connection : private boost::noncopyable {
public:
  typedef boost::shared_ptr<data_connection> sptr;
  typedef boost::shared_ptr<boost::asio::ip::tcp::socket> tcp_socket_ptr;
  typedef std::vector<char> Data;
  typedef boost::shared_ptr<Data> data_ptr;
  typedef std::deque<data_ptr> ListOfPackets;

  data_connection(boost::asio::io_service& io_service,
                  boost::asio::strand& strand,
                  tcp_socket_ptr p)
    : io_service_(io_service)
    , strand_(strand)
    , tcp_socket_ptr_(p)
    , isOpen_(true) {}

  ~data_connection() {
    close();
  }

  void close() {  tcp_socket_ptr_->close(); isOpen_= false; }
  bool is_open() const { return isOpen_; }

  bool push_back(const Data& d) { 
    // TODO check length of listOfPackets_
    if (is_open()) {
      const bool listOfPacketsWasEmpty(isEmpty());
      // std::cout << "push_back " << (listOfPacketsWasEmpty ? "empty " : "non-empty ")
      //           << listOfPackets_.size() << std::endl;
      listOfPackets_.push_back(data_ptr(new Data(d))); 
      if (listOfPacketsWasEmpty)
        async_write();
      return true;
    }
    return false;
  }
  bool isEmpty() const { 
    return listOfPackets_.empty(); 
  }
  void pop_front() { 
    listOfPackets_.pop_front(); 
  }
  ListOfPackets::const_reference front() const { 
    return listOfPackets_.front(); 
  }

  void async_write() {
    if (!isEmpty()) {
      ListOfPackets::const_reference dataPtr(front());
      boost::asio::async_write(*tcp_socket_ptr_,
                               boost::asio::buffer(&dataPtr->front(), dataPtr->size()),
                               strand_.wrap(boost::bind(&data_connection::handle_write, 
                                                        this,
                                                        boost::asio::placeholders::error,
                                                        boost::asio::placeholders::bytes_transferred)));
    }
  }
  
  void handle_write(const boost::system::error_code& error,
                    std::size_t bytes_transferred) {
    if (error) {
      close();
    } else {
      pop_front();
      async_write();
    }
  }

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
  boost::asio::strand&     strand_;
  tcp_socket_ptr           tcp_socket_ptr_;
  bool                     isOpen_;
  ListOfPackets            listOfPackets_;
} ;

class server : private boost::noncopyable {
public:
  typedef boost::shared_ptr<server> sptr;
  typedef boost::shared_ptr<boost::asio::ip::tcp::socket> tcp_socket_ptr;
  typedef std::set<tcp_socket_ptr> Sockets;
  typedef boost::shared_ptr<data_connection> data_connection_ptr;
  typedef std::set<data_connection_ptr> data_connections;
  typedef boost::posix_time::ptime ptime;
  typedef boost::posix_time::time_duration time_duration;

  server(boost::asio::io_service& io_service,
         const boost::asio::ip::tcp::endpoint& endpoint_ctrl,
         const boost::asio::ip::tcp::endpoint& endpoint_data,
         Perseus::ReceiverPtr recPtr,
         const boost::property_tree::ptree& config)
    : io_service_(io_service)
    , strand_(io_service)
    , acceptor_ctrl_(io_service, endpoint_ctrl)
    , acceptor_data_(io_service, endpoint_data)
    , recPtr_(recPtr)
    , usbBufferSize_(config.get<unsigned>("perseus.USBBufferSize"))
    , sampleNumber_(0)
    , dtCallback_(0,0,0,
                  usbBufferSize_/6*time_duration::ticks_per_second()/recPtr_->sampleRate()) {
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
    // PTimeLowpassFilters:
    {
      using boost::property_tree::ptree;
      const ptree& pt(config.get_child("perseus.CascadedPTimeLowPass"));
      std::cout << "Cascaded Lowpass Filters: " << std::endl;
      const double dtCallbackSec(double(dtCallback_.ticks())/time_duration::ticks_per_second());
      BOOST_FOREACH(const ptree::value_type& filter, config.get_child("perseus.CascadedPTimeLowPass")) {
        if (filter.first != "Tau_Sec") {
          std::cout << "  + unknown key: [" << filter.first << "]" << std::endl;
          continue;
        }
        const double filterTimeconstant(boost::lexical_cast<double>(filter.second.data()));
        std::cout << "  + Lowpass Filter " << filter.first << "=" << filterTimeconstant << std::endl;
        ptimeFilter_.add(Filter::PTimeLowPass::make(dtCallbackSec, filterTimeconstant));
        dtFilter_.add(Filter::LowPass<double>::make(dtCallbackSec, filterTimeconstant));        
      }
      const ptime now(boost::posix_time::microsec_clock::universal_time());
      ptimeFilter_.init(now, now);
      dtFilter_.init(now, dtCallbackSec);
      ptimeOfCallback_ = now;
    }
    recPtr_->startAsyncInput(usbBufferSize_, server::receiverCallback, this);
    std::cout << "ptime_ = " << ptimeOfCallback_ << " " 
              << "dt= " << 1e-6*dtCallback_.total_microseconds() << std::endl;
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
      data_connections_.insert(data_connection_ptr(new data_connection(io_service_, strand_, socket)));
      tcp_socket_ptr new_socket(new boost::asio::ip::tcp::socket(acceptor_data_.get_io_service()));
      acceptor_data_.async_accept(*new_socket,
                                  strand_.wrap(boost::bind(&server::handle_accept_data, this,
                                                           boost::asio::placeholders::error, new_socket)));
    } else {
      // error 
    }
  }

  Header getHeader(const unsigned nSamples, ptime approxPTime) {
    return Header((sampleNumber_+=nSamples) - nSamples,
                  recPtr_->sampleRate(),
                  recPtr_->ddcCenterFrequency(),
                  nSamples,
                  0, // TODO
                  recPtr_->attenId(),
                  recPtr_->enablePresel(),
                  recPtr_->enablePreamp(),
                  recPtr_->enableDither(),
                  approxPTime);
  }

  static int receiverCallback(void *buf, int buf_size, void *extra) {
    server* sp= (server* )extra;
    const unsigned nSamples = buf_size/6;

    const ptime now(boost::posix_time::microsec_clock::universal_time());
    const time_duration dt(now - sp->ptimeOfCallback_);

    const ptime oldFilterTime(sp->ptimeFilter_.x());
    const bool doInterpolation(std::abs(dt.ticks() - sp->dtCallback_.ticks()) > sp->dtCallback_.ticks()/10); 
    const ptime nowInterpolated(doInterpolation ? oldFilterTime + sp->dtCallback_ : now);
    sp->ptimeFilter_.update(nowInterpolated, nowInterpolated);
    sp->dtFilter_.update(nowInterpolated, (doInterpolation 
                                           ? double(sp->dtCallback_.ticks())
                                           : double(dt.ticks())) / time_duration::ticks_per_second());
    sp->ptimeOfCallback_= now;
#if 1
    std::cout << "receiverCallback "
              << sp->ptimeOfCallback_ << " "
              << sp->ptimeFilter_.x() << " "
              << dt << " "
              << sp->dtCallback_ << " "
              << (sp->ptimeFilter_.x()-oldFilterTime).ticks() << " " << sp->dtCallback_.ticks() << " "
              << sp->dtFilter_.x() << " " 
              << sp->dtFilter_.x() / double(sp->dtCallback_.ticks())*time_duration::ticks_per_second()-1. << " "
              << sp->ptimeOfCallback_ - sp->ptimeFilter_.x() << " "
              << ((std::abs(dt.ticks() - sp->dtCallback_.ticks()) < sp->dtCallback_.ticks()/10) ? "OK" : "IP") 
              <<  std::endl;
#endif
    const Header header(sp->getHeader(nSamples, oldFilterTime));
#if 1
    std::vector<char> dataVector;
    std::copy((char*)&header, (char*)&header+sizeof(Header), std::back_inserter(dataVector));
    std::copy((char*)buf,     (char*)buf+buf_size,           std::back_inserter(dataVector));

    sp->io_service_.dispatch(boost::bind(&server::sendDataToClients, sp, dataVector));
#endif

    boost::system::error_code ec;
    for (unsigned u(0); sp->io_service_.poll(ec) > 0 && u < 10000; ++u)
      if (ec) break;
    return 0;
  }
  static void sendDataToClients(server* sp, const std::vector<char>& dataVector) {
    if (sp->sampleNumber_ % 85000 == 0)
      std::cout << "data_connections_.size*() = " << sp->data_connections_.size() 
                << " sampleNumber=" << sp->sampleNumber_ << std::endl;
    for (data_connections::iterator i(sp->data_connections_.begin()); i!=sp->data_connections_.end();) {
      if ((*i)->push_back(dataVector))
        ++i;
      else
        sp->data_connections_.erase(i++);
    }
  }

  void stop() {
    recPtr_->stopAsyncInput();
    io_service_.post(boost::bind(&server::do_stop, this));
    // process the remaining actions
    io_service_.run();
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
  time_duration                  dtCallback_;
  ptime                          ptimeOfCallback_;
  Filter::Cascaded<ptime>        ptimeFilter_;
  Filter::Cascaded<double>       dtFilter_;
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
    server::sptr sp;
    {
      wait_for_signal w;
      w.add_signal(SIGINT)
       .add_signal(SIGQUIT)
       .add_signal(SIGTERM);
      sp = server::sptr(new server(io_service,
                                   boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 
                                                                  pt.get<unsigned>("server.ctrl.port")),
                                   boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 
                                                                  pt.get<unsigned>("server.data.port")),
                                   pp,
                                   pt));
    }
    sp->stop();
    // run(io_service, s);    
  } catch (std::exception &e) {
    std::cout << "Error: " << e.what() << "\n";
  }
  return 0;
}
