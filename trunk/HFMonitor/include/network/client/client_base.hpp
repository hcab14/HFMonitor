// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2013 Christoph Mayer
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
#ifndef _CLIENT_BASE_HPP_cm121230_
#define _CLIENT_BASE_HPP_cm121230_

#include <iostream>
#include <set>
#include <sstream>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>

#include "logging.hpp"
#include "network/broadcaster/directory.hpp"
#include "network/protocol.hpp"
#include "processor.hpp"

namespace network {
  namespace client {

    // generic base class for connecting to a broadcaster
    class client_base : public boost::noncopyable {
    public:
      enum received_data_type {
        received_header, 
        received_data
      } ;

      typedef processor::base::data_buffer_type data_buffer_type;

      client_base(boost::asio::io_service& io_service,
                  const boost::property_tree::ptree& config)
        : io_service_(io_service)
        , strand_(io_service)
        , socket_(io_service)
        , tick_buffer_('a')
        , timer_(io_service, boost::posix_time::seconds(1)) {
        if (!connect_socket(config.get<std::string>("server.<xmlattr>.host"),
                            config.get<std::string>("server.<xmlattr>.port")))
          throw std::runtime_error("connect failed");
      }

      virtual ~client_base() {
        LOG_INFO("~client_base: close");
        socket_.close();
      }

      // access methods
      boost::asio::io_service&      get_io_service()      { return io_service_; }
      boost::asio::strand&          get_strand()          { return strand_; }  
      const protocol::header&       get_header()    const { return header_; }
      const broadcaster::directory& get_directory() const { return directory_; }

      // blocking "ls"
      std::set<std::string> ls() {
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

      // blocking "GET" command
      //  * names can be a list (separated by white space) of names
      //  * a name can also be a perl regular expression
      bool connect_to(std::string names) {
        std::istringstream iss(send_request(str(boost::format("GET %s") % names)));
        std::string f;
        if (iss >> f) {
          LOG_INFO(str(boost::format("clien_base::connect_to '%s'") % f));
          if (f == "OK") {
            return true;
          } else {
            LOG_INFO(str(boost::format("clien_base::connect_to response: '%s' '%s'") % f % names));
            return false;
          }
        } else {
          return false;
        }
      }

      void start() {
        async_receive_header();
        timer_.async_wait(get_strand().wrap(boost::bind(&client_base::on_tick, this)));    
      }

      // stop the client in a nice way
      void stop() { get_io_service().post(get_strand().wrap(boost::bind(&client_base::do_close, this))); }

      // this method is to be overwitten in a derived class
      //  * get_header() provides the header
      virtual void process(data_buffer_type::const_iterator begin,
                           data_buffer_type::const_iterator end) {
        LOG_INFO(str(boost::format("process: h='%s', length=%d")
                     % get_header()
                     % std::distance(begin,end)));
      }

      // notifies derived classes of an update of directory
      virtual void directory_update(const broadcaster::directory&) {
        // NOP by default
      }

    protected:
      virtual void dump_wav(processor::service_base::ptime t, std::string str_name,
                            data_buffer_type::const_iterator begin,
                            data_buffer_type::const_iterator end) {
        // to be overwritten in a derived class
      }

    private:
      void do_close() {
        LOG_INFO("do_close");
        socket_.close();
      }

      void on_tick() {
        // send tick to server
        boost::asio::async_write(socket_,
                                 boost::asio::buffer(&tick_buffer_, sizeof(tick_buffer_)),
                                 get_strand().wrap(boost::bind(&client_base::handle_write_tick,
                                                               this,
                                                               boost::asio::placeholders::error,
                                                               boost::asio::placeholders::bytes_transferred,
                                                               tick_buffer_)));
        timer_.expires_at(timer_.expires_at() + boost::posix_time::seconds(1));
        timer_.async_wait(get_strand().wrap(boost::bind(&client_base::on_tick, this)));
        tick_buffer_ = (tick_buffer_ != 'z') ? tick_buffer_+1 : 'a';
      }
      void handle_write_tick(boost::system::error_code ec,
                             std::size_t bytes_transferred,
                             char tick_value) {
        if (ec || bytes_transferred == 0) {
          LOG_WARNING(str(boost::format("handle_write_tick aborting: error '%s' bytes_transferred=%d")
                          % ec.message()
                          % bytes_transferred));
          get_io_service().stop();
          do_close();
        } else
          LOG_INFO(str(boost::format("tick '%c' sent") % tick_value));
      }

      // asyncronuosly receive the header of a data packet
      void async_receive_header() {
        boost::asio::async_read(socket_,
                                boost::asio::buffer(&header_, sizeof(protocol::header)),
                                get_strand().wrap(boost::bind(&client_base::on_receive,
                                                              this,
                                                              received_header,
                                                              boost::asio::placeholders::error,
                                                              boost::asio::placeholders::bytes_transferred)));
      }
  
      // asyncronuosly receive the data of a data packet
      void async_receive_data() {
        if (header_.length() >= data_buffer().size())
          LOG_ERROR(str(boost::format("header: %s H: %d D: %d") % header_ % header_.length() % data_buffer().size()));
        ASSERT_THROW(header_.length() < data_buffer().size());
        boost::asio::async_read(socket_,
                                boost::asio::buffer(&data_buffer(), header_.length()),
                                get_strand().wrap(boost::bind(&client_base::on_receive,
                                                              this,
                                                              received_data,
                                                              boost::asio::placeholders::error,
                                                              boost::asio::placeholders::bytes_transferred)));
      }

      // receive callback
      void on_receive(received_data_type rdt,
                      boost::system::error_code ec,
                      std::size_t bytes_transferred) {
        if (ec) {
          LOG_INFO(str(boost::format("receive error: %s. Aborting") % ec.message()));
          stop();
          return;
        }
        switch (rdt) {
        case received_header:
          async_receive_data();
          break;
        case received_data:
          assert(bytes_transferred == header_.length());
          if (header_.id() == broadcaster::directory::id()) {
            broadcaster::directory new_directory;
            new_directory.update(header_.length(), data_buffer().data());
            directory_update(new_directory);
            std::swap(new_directory, directory_);
          } else {
            // dump all wav stream
            const std::string str_name(get_directory().stream_name_of(get_header().stream_number()));
            if (get_header().id() == "WAV_0000")
              dump_wav(get_header().approx_ptime(), str_name, data_buffer().begin(), data_buffer().begin()+header_.length());

            // process data samples in a method overwritten in a derived class
            process(data_buffer().begin(), data_buffer().begin()+header_.length());
          }
          // then receive the next header
          async_receive_header();
          break;
        default:
          throw std::runtime_error("client_base::on_receive: invalid received data type");
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
        // std::istream is(&response_streambuf);
        // std::string line;
        // std::getline(is, line);
        // return line;
        std::istream buffer(&response_streambuf);
        std::stringstream string_buffer;    
        buffer >> string_buffer.rdbuf();
        LOG_INFO(str(boost::format("response: '%s'") % string_buffer.str()));
        return string_buffer.str();
      }

      // blocking connect
      bool connect_socket(std::string server_name,
                          std::string server_port) {
        using boost::asio::ip::tcp;
        tcp::resolver resolver(get_io_service());
        tcp::resolver::query    query(server_name, server_port);
        tcp::resolver::iterator endpoint_iterator(resolver.resolve(query));
        tcp::resolver::iterator end;
        boost::system::error_code error = boost::asio::error::host_not_found;
        while (error && endpoint_iterator != end) {
          socket_.close();
          socket_.connect(*endpoint_iterator++, error);
          LOG_INFO(str(boost::format("error(connect)= '%s'") % error));
        }
        return (!error) || (endpoint_iterator != end);
      }

      static data_buffer_type& data_buffer() {
        static data_buffer_type db;
        return db;
      }

      boost::asio::io_service&          io_service_;
      boost::asio::strand               strand_;
      boost::asio::ip::tcp::socket      socket_;
      protocol::header                  header_;
      char                              tick_buffer_;
      boost::asio::deadline_timer       timer_;
      broadcaster::directory            directory_;
    } ;

  } // namespace client
} // namespace network
#endif // _CLIENT_BASE_HPP_cm121230_
