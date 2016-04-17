// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2014 Christoph Mayer
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
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
#include "network.hpp"
#include "network/broadcaster/directory.hpp"
#include "network/broadcaster/data_connection.hpp"

namespace network {
  // -----------------------------------------------------------------------------
  // broadcaster
  //  * uses boost::asio
  //  * the data is sent to all connected clients
  //  * being-alive-ticks are exchanged between server and client
  //  * config: 
  //   <Data port="xxxx"
  //         maxQueueSize_MB="xxxx"
  //         maxQueueDelay_Minutes="xxxx"></Data>
  namespace broadcaster {
  class broadcaster : private boost::noncopyable {
  public:
    typedef boost::shared_ptr<broadcaster> sptr;

    static sptr make(const boost::property_tree::ptree& config) {
      return sptr(new broadcaster(config));
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
      network::get_io_service().post(strand_.wrap(boost::bind(&broadcaster::do_stop, this)));
    }

    typedef data_connection::ptime ptime;
    typedef std::string data_type;

    // returns the unique stream number corresponding to id
    boost::uint32_t register_stream(std::string path) {
      if (path == "") return 0;
      const std::pair<boost::uint32_t, bool> r(directory_.insert(path));
      // if the directory has changed, broadcast the new directory to all clients
      if (r.second)
        bc_directory();
      return r.first;
    }

    void bc_directory() {
      const ptime now(boost::posix_time::microsec_clock::universal_time());
      bc_data(now, "", directory_.id(), directory_.serialize(now));
    }

    typedef boost::shared_ptr<data_type> data_ptr_type;

    // broadcast data
    void bc_data(ptime t,
                 std::string path,
                 data_type id,
                 data_type data,
                 data_type preamble="") {
      {
        const ptime now(boost::posix_time::microsec_clock::universal_time());
        LOG_INFO(str(boost::format("t=%s now=%s path=%s") % t % now % path));
      }
      // status of broadcaster -> log file
      log_status(t);

      // create a shared pointer of the to-be-broadcasted data
      data_ptr_type bytes_data(data_ptr_type(new data_type(add_header(t, path, id ,data))));
      data_ptr_type bytes_preamble(preamble != ""
                                   ? data_ptr_type(new data_type(add_header(t, path, id, preamble)))
                                   : data_ptr_type());

      // insert to all data connections, checking if they are still alive
      for (std::set<data_connection::sptr>::iterator k(data_connections_.begin());
           k!=data_connections_.end(); ) {
        if ((*k)->push_back(t, path, bytes_data, bytes_preamble))
          ++k;
        else
          data_connections_.erase(k++);
      }
    }

    boost::asio::strand& get_strand() { return strand_; }

  protected:
    data_type add_header(ptime t,
                         std::string path,
                         data_type id,
                         data_type data) {
      const protocol::header h(id, t, register_stream(path), data.size());
#if 1
      data_type bytes(sizeof(protocol::header)+data.size(), 0);
      std::copy(h.begin(),    h.end(),    bytes.begin());
      std::copy(data.begin(), data.end(), bytes.begin()+sizeof(protocol::header));
#else
      data_type bytes;
      std::copy(h.begin(),    h.end(),    std::back_inserter(bytes));
      std::copy(data.begin(), data.end(), std::back_inserter(bytes));
#endif
      return bytes;
    }
  private:
    broadcaster(const boost::property_tree::ptree& config)
      : strand_(network::get_io_service())
      , max_queue_total_size_(1024*1024*config.get<size_t>("Data.<xmlattr>.maxQueueSize_MB"))
      , max_queue_delay_(boost::posix_time::minutes(config.get<size_t>("Data.<xmlattr>.maxQueueDelay_Minutes")))
      , last_log_status_time_(boost::posix_time::microsec_clock::universal_time())
    {
      using namespace boost::asio::ip;
      acceptor_map_["Data"] = acceptor_ptr
        (new tcp::acceptor(network::get_io_service(),
                           tcp::endpoint(tcp::v4(), config.get<size_t>("Data.<xmlattr>.port"))));
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
      tcp_socket_ptr new_socket(new boost::asio::ip::tcp::socket(network::get_io_service()));
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
          data_connections_.insert(data_connection::make(network::get_io_service(), strand_, socket,
                                                         directory_, max_queue_total_size_, max_queue_delay_));

          // asynchronously accept next data connection
          async_accept(a);

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
                              % dc->remote_endpoint(ecr_ignore) 
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

    boost::asio::strand     strand_;               // asio strand
    acceptor_map            acceptor_map_;         // map of acceptors
    const size_t            max_queue_total_size_; // queue total size of data connections
    const time_duration     max_queue_delay_;      // queue maximum delay of data connections
    directory               directory_;            // directory of available data streams
    data_connections        data_connections_;     // set of data connections
    mutable ptime           last_log_status_time_; // 
  } ;

} // namespace broadcaster
} // namespace network
#endif // _BROADCASTER_HPP_cm111219_

