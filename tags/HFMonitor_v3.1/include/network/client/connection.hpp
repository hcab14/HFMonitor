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
#ifndef _CLIENT_CONNECTION_HPP_cm140509_
#define _CLIENT_CONNECTION_HPP_cm140509_

#include <stdlib.h>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/mutex.hpp>

#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>

#include "logging.hpp"
#include "network/broadcaster/directory.hpp"
#include "network/protocol.hpp"
#include "processor.hpp"

namespace network {
  namespace client {
  
    // base class for client connections 
    //  * uses stackful coroutines (boost::asio::spawn)
    //  * should at some point replace network::client::client_base
    //
    class connection : public  boost::enable_shared_from_this<connection>
                     , private boost::noncopyable {
    public:
      typedef boost::shared_ptr<connection> sptr;
      typedef processor::base::data_buffer_type data_buffer_type;

      static sptr make(boost::asio::io_service& io_service,
                       std::string host,
                       std::string port) {
        return sptr(new connection(io_service, host, port));
      }

      virtual ~connection() {
        stop();
      }
 
      // access methods
      boost::asio::io_service&      io_service()         { return strand_.get_io_service(); }
      boost::asio::strand&          strand()             { return strand_; }  
      const protocol::header&       header()    const    { return header_; }
      const broadcaster::directory& directory() const    { return directory_; }
      bool                          is_connected() const { return socket_.is_open(); }


      // starts data stream processing
      void start(std::string streams) {
        boost::asio::spawn(strand_,
                           boost::bind(&connection::do_start,
                                       shared_from_this(), streams, _1));
        // start the deadline actor
        boost::asio::spawn(strand_,
                           boost::bind(&connection::check_deadline,
                                       shared_from_this(), _1));
      }
      void do_start(std::string streams, boost::asio::yield_context yield) {
        std::string response(send_request(str(boost::format("GET %s") % streams)));
        LOG_INFO(str(boost::format("connect_to response: '%s' '%s'") % response % streams));
        ASSERT_THROW(response == "OK");
        boost::asio::spawn(strand_,
                           boost::bind(&connection::receive_data,
                                       shared_from_this(), _1));
        boost::asio::spawn(strand_,
                           boost::bind(&connection::heartbeat,
                                       shared_from_this(), _1));
      }
      // stop
      void stop() {
        stopped_ = true;
        socket_.close();
        deadline_.cancel();
        heartbeat_timer_.cancel();
      }    

      // blocking "ls"
      std::set<std::string> ls() {
        ASSERT_THROW(!stopped_ && socket_.is_open());

        std::set<std::string> streams;
        std::istringstream iss(send_request("LIST"));
        std::string     stream_name;
        boost::uint32_t stream_number;
        bool ok(true);
        while (ok) {
          if (iss >> stream_name) {
            if (iss >> stream_number) {
              if (stream_name != "")
                streams.insert(stream_name);
            } else ok= false;
          } else ok= false;
        }
        return streams;
      }
    
    protected:    
      // this method is to be overwitten in a derived class
      //  * header() provides the header
      virtual void process(data_buffer_type::const_iterator begin,
                           data_buffer_type::const_iterator end) {
        LOG_INFO(str(boost::format("process: h='%s', length=%d")
                     % header()
                     % std::distance(begin,end)));
      }
    
      // notifies derived classes of an update of directory
      virtual void directory_update(const broadcaster::directory&) {
        // NOP by default
      }
    
      virtual void dump_wav(processor::service_base::ptime t, std::string str_name,
                            data_buffer_type::const_iterator begin,
                            data_buffer_type::const_iterator end) {
        // to be overwritten in a derived class
      }
    
      connection(boost::asio::io_service& io_service,
                 std::string host,
                 std::string port)
        : stopped_(false)
        , strand_(io_service)
        , resolver_(io_service)
        , socket_(io_service)
        , tick_buffer_('a')
        , deadline_(io_service)
        , heartbeat_timer_(io_service) {
        ASSERT_THROW(true == connect_socket(host, port));
      }

    private:    
      // blocking connect
      bool connect_socket(std::string host,
                          std::string port) {
        using boost::asio::ip::tcp;
        tcp::resolver resolver(io_service());
        const tcp::resolver::query query(host, port);
        tcp::resolver::iterator endpoint_iterator(resolver.resolve(query));
        tcp::resolver::iterator end;
        boost::system::error_code error(boost::asio::error::host_not_found);
        while (error && endpoint_iterator != end) {
          socket_.close();
          socket_.connect(*endpoint_iterator++, error);
          LOG_INFO(str(boost::format("error(connect)= '%s'") % error));
        }
        if (error) {
          tcp::endpoint tcp(boost::asio::ip::address::from_string(name), atoi(port.c_str()));
          socket_.close();
          socket_.connect(tcp, error);
          LOG_INFO(str(boost::format("error(connect)= '%s'") % error));
        }
        return (!error);// || (endpoint_iterator != end);
      }

      // data processing
      void receive_data(boost::asio::yield_context yield) {
        try {      
          while (1) {
            deadline_.expires_from_now(boost::posix_time::seconds(10));
            const std::size_t n_header(boost::asio::async_read
                                       (socket_, boost::asio::buffer(&header_, sizeof(protocol::header)), yield));
            if (!socket_.is_open()) // timeout
              throw std::runtime_error("async_read timeout");
            ASSERT_THROW(n_header == sizeof(network::protocol::header));
          
            if (header_.length() >= data_buffer_.size())
              LOG_ERROR(str(boost::format("header: %s H: %d D: %d") % header_ % header_.length() % data_buffer_.size()));
            ASSERT_THROW(header_.length() < data_buffer_.size());
          
            deadline_.expires_from_now(boost::posix_time::seconds(10));
            const std::size_t n_data(boost::asio::async_read
                                     (socket_, boost::asio::buffer(&data_buffer_, header_.length()), yield));
            if (!socket_.is_open()) // timeout
              throw std::runtime_error("async_read timeout");
            ASSERT_THROW(n_data == header_.length());

            if (header_.id() == broadcaster::directory::id()) {
              broadcaster::directory new_directory;
              new_directory.update(header_.length(), data_buffer_.data());
              directory_update(new_directory);
              std::swap(new_directory, directory_);
            } else {
              // dump all wav streams
              const std::string str_name(directory().stream_name_of(header().stream_number()));
              if (header().id() == "WAV_0000")
                dump_wav(header().approx_ptime(), str_name, data_buffer_.begin(), data_buffer_.begin()+header_.length());
            
              LOG_INFO(str(boost::format("___ process: %s h='%s'") % str_name % header_));

              // process data samples in a method overwritten in a derived class
              process(data_buffer_.begin(), data_buffer_.begin()+header_.length());
            }
          }
        } catch (std::exception& e) {
          LOG_ERROR(e.what());
        }
      }

      // blocking send request, returns the response
      std::string send_request(std::string request) {
        LOG_INFO(str(boost::format("send_request: '%s'") % request));
        request += "\r\n";
        boost::asio::streambuf request_streambuf;
        std::ostream request_stream(&request_streambuf);
        request_stream << request;
        boost::asio::write(socket_, request_streambuf);
        boost::asio::streambuf response_streambuf;
        boost::asio::read_until(socket_, response_streambuf, "\r\n");

        std::istream buffer(&response_streambuf);
        std::stringstream string_buffer;    
        buffer >> string_buffer.rdbuf();

        std::string response(string_buffer.str());
        response.erase(response.find_last_not_of("\r\n")+1);

        LOG_INFO(str(boost::format("response: '%s'") % response));
        return response;
      }

      void check_deadline(boost::asio::yield_context yield) {
        try {
          while (socket_.is_open()) {
            if (stopped_)
              return;
            boost::system::error_code ignored_ec;
            deadline_.async_wait(yield[ignored_ec]);
            if (deadline_.expires_from_now() <= boost::posix_time::seconds(0))
              socket_.close();
          }
        } catch (std::exception& e) {
          LOG_ERROR(e.what());
        }
      }

      void heartbeat(boost::asio::yield_context yield) {
        try {
          while (socket_.is_open()) {
            if (stopped_)
              return;
            heartbeat_timer_.expires_from_now(boost::posix_time::seconds(1));        
            const size_t n(boost::asio::async_write
                           (socket_, boost::asio::buffer(&tick_buffer_, sizeof(tick_buffer_)), yield));
            ASSERT_THROW(n == sizeof(tick_buffer_));
            tick_buffer_ = (tick_buffer_ != 'z') ? tick_buffer_+1 : 'a';
        
            boost::system::error_code ignored_ec;
            heartbeat_timer_.async_wait(yield[ignored_ec]);
            if (deadline_.expires_from_now() <= boost::posix_time::seconds(0))
              socket_.close();        
          }
        } catch (std::exception& e) {
          LOG_ERROR(e.what());
        }
      }

      bool                           stopped_;
      boost::asio::strand            strand_;
      boost::asio::ip::tcp::resolver resolver_;
      boost::asio::ip::tcp::socket   socket_;
      protocol::header               header_;
      data_buffer_type               data_buffer_;
      char                           tick_buffer_;
      boost::asio::deadline_timer    deadline_;
      boost::asio::deadline_timer    heartbeat_timer_;
      broadcaster::directory         directory_;
    } ;

  } // namespace client
} // namespace network

#endif // _CLIENT_CONNECTION_HPP_cm140509_
