// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _BROADCASTER_HPP_cm111219_
#define _BROADCASTER_HPP_cm111219_

#include <iostream>
#include <set>
#include <string>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>

#include "logging.hpp"
#include "network/broadcaster/directory.hpp"
#include "network/broadcaster/data_connection.hpp"

// -----------------------------------------------------------------------------
// broadcaster
//  * uses boost::asio
//  * the data is sent to all connected clients
//  * being-alive-ticks are exchanged between server and client
//  * config: 
//   <Data port="xxxx"
//         maxQueueSize_MB="xxxx"
//         maxQueueDelay_Minutes="xxxx"></Data>
//   <Ctrl port="xxxx"></Ctrl>

class broadcaster : private boost::noncopyable {
public:
  typedef boost::shared_ptr<broadcaster> sptr;

  static sptr make(boost::asio::io_service& io_service,
                   const boost::property_tree::ptree& config) {
    return sptr(new broadcaster(io_service, config));
  }
  virtual ~broadcaster() {}

  // start broadcaster
  void start() {
    // start listening on data/ctrl sockets
    LOG_INFO("broadcaster start");
    start_listen();
  }
  
  // stop broadcaster
  void stop() {
    // announce stop of io_service
    LOG_INFO("broadcaster stop");
    get_io_service().post(strand_.wrap(boost::bind(&broadcaster::do_stop, this)));
  }

  // returns a reference to the io service object
  boost::asio::io_service& get_io_service() { return strand_.get_io_service(); }

  typedef data_connection::ptime ptime;
  typedef std::string data_type;

  // returns the unique stream number corresponding to id
  boost::uint32_t register_stream(std::string path) {
    const std::pair<boost::uint32_t, bool> r(directory_.insert(path));
    // if the directory has changed, broadcast the new directory to all clients
    if (r.second)
      bc_directory();
    return r.first;
  }

  void bc_directory() {
    const ptime now(boost::posix_time::microsec_clock::universal_time());
    bc_data(now, "", directory_.serialize(now));
  }

  // broadcast data
  void bc_data(ptime t,
               std::string path,
               data_type d,
               data_type preamble="") {
    // status of broadcaster -> log file
    log_status(t);

    // create a shared pointer of the to-be-broadcasted data
    boost::shared_ptr<data_type> dp(new data_type(d));
    
    // insert to all data connections, checking if they are still alive
    for (std::set<data_connection::sptr>::iterator k(data_connections_.begin());
         k!=data_connections_.end(); ) {
      if ((*k)->push_back(t, path, dp, preamble))
        ++k;
      else
        data_connections_.erase(k++);
    }
  }

protected:
  // returns a reference to the strand object
  boost::asio::strand& get_strand() { return strand_; }

private:
  broadcaster(boost::asio::io_service& io_service,
              const boost::property_tree::ptree& config)
    : strand_(io_service)
    , max_queue_total_size_(1024*1024*config.get<size_t>("Data.<xmlattr>.maxQueueSize_MB"))
    , max_queue_delay_(boost::posix_time::minutes(config.get<size_t>("Data.<xmlattr>.maxQueueDelay_Minutes")))
    , last_log_status_time_(boost::posix_time::microsec_clock::universal_time())
  {
    using namespace boost::asio::ip;
    acceptor_map_["Data"] = acceptor_ptr
      (new tcp::acceptor(io_service, tcp::endpoint(tcp::v4(), config.get<size_t>("Data.<xmlattr>.port"))));
    acceptor_map_["Ctrl"] = acceptor_ptr
      (new tcp::acceptor(io_service, tcp::endpoint(tcp::v4(), config.get<size_t>("Ctrl.<xmlattr>.port"))));
  }

  typedef data_connection::time_duration time_duration;
  typedef boost::shared_ptr<boost::asio::ip::tcp::socket> tcp_socket_ptr;
  typedef std::set<data_connection::sptr> data_connections;
  typedef boost::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor_ptr;
  typedef std::map<std::string, acceptor_ptr> acceptor_map;

  void start_listen() {
    BOOST_FOREACH(const acceptor_map::value_type& a, acceptor_map_) {
      // setup acceptor
      a.second->set_option(boost::asio::socket_base::reuse_address(true));

      // start listening
      a.second->listen();      

      // asynchronously accept data connections
      async_accept(a);
    }
  }

  void async_accept(const acceptor_map::value_type& a) {
    tcp_socket_ptr new_socket(new boost::asio::ip::tcp::socket(a.second->get_io_service()));
    a.second->async_accept(*new_socket,
                           strand_.wrap(boost::bind(&broadcaster::handle_accept, this,
                                                    boost::asio::placeholders::error,
                                                    new_socket, a)));
  }

  void handle_accept(const boost::system::error_code& ec,
                     tcp_socket_ptr socket,
                     const acceptor_map::value_type& a) {
    LOG_INFO(str(boost::format("servce::handle_accept type='%s' error_code='%s'") % a.first % ec));
    if (!ec) {
      if (a.first == "Data") {
        boost::system::error_code ec_ignore;
        LOG_INFO(str(boost::format("remote endpoint='%s'") % socket->remote_endpoint(ec_ignore)));

        // make new data connection and insert into the set of open connections
        data_connections_.insert(data_connection::make(get_io_service(), strand_, socket,
                                                       directory_, max_queue_total_size_, max_queue_delay_));

        // asynchronously accept next data connection
        async_accept(a);

      } else if (a.first == "Ctrl") {
        // do nothing for now
      } else {
        LOG_ERROR(str(boost::format("unknown type='%s'") % a.first));
      }
    } else {
      LOG_ERROR(str(boost::format("handle_accept type='%s' ec='%s'") % a.first % ec));
    }
  }

  void log_status(ptime t) const {
    if (t-last_log_status_time_ > boost::posix_time::seconds(1)) {
      LOG_STATUS_T(t, str(boost::format("#connections=%3d") % data_connections_.size()));
      BOOST_FOREACH(const data_connections::value_type& dc, data_connections_) {
        boost::system::error_code ecl_ignore, ecr_ignore;
        LOG_STATUS_T(t, str(boost::format("  %s - %s : delay[ms] = %.2f (%s)")
                            % dc->local_endpoint(ecl_ignore) 
                            % dc->local_endpoint(ecr_ignore) 
                            % dc->max_delay_msec()
                            % dc->stream_name()));
        dc->reset_max_delay();
      }
      last_log_status_time_= t;
    }
  }

  void do_stop() {
    data_connections_.clear();
  }

  boost::asio::strand   strand_;               // asio strand, keeps io service
  acceptor_map          acceptor_map_;         // map of acceptors
  const size_t          max_queue_total_size_; // queue total size of data connections
  const time_duration   max_queue_delay_;      // queue maximum delay of data connections
  broadcaster_directory directory_;            // directory of available data streams
  data_connections      data_connections_;     // set of data connections
  mutable ptime         last_log_status_time_; // 
} ;

#endif // _BROADCASTER_HPP_cm111219_

