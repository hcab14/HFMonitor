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
#ifndef _DATA_CONNECTION_HPP_cm111219_
#define _DATA_CONNECTION_HPP_cm111219_

#include <deque>
#include <iostream>
#include <iterator>
#include <string>
#include <sstream>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/regex.hpp>

#include "logging.hpp"
#include "network/broadcaster/directory.hpp"

// -----------------------------------------------------------------------------
// data_connection
//  * buffered data stream
//  * all replys and requests are terminated by "\r\n"
//  * protocol:
//     - "LIST" : list directory of available streams
//     - "GET [stream_name]" : get 'stream_name', from then on data will be sent
//  * when (binary) data is being sent, a "tick" protocol ensures that the
//    connection is still there

class data_connection : private boost::noncopyable {
public:
  typedef boost::posix_time::ptime ptime;
  typedef boost::posix_time::time_duration time_duration;
  
  typedef boost::shared_ptr<data_connection> sptr;
  typedef boost::shared_ptr<boost::asio::ip::tcp::socket> tcp_socket_ptr;
  typedef std::string data_type;
  typedef boost::shared_ptr<data_type> data_ptr;
  typedef std::pair<ptime, data_ptr> ptime_data_pair;
  typedef std::deque<ptime_data_pair> list_of_packets;
  
  enum status_type {
    status_error      = -1, //
    status_init       =  0, // negotiation phase: client asks for streams
    status_configured =  1  // after streams have been selected, data is sent to the client
  } status;
  
public:
  data_connection(boost::asio::io_service& io_service,
                  boost::asio::strand& strand,
                  tcp_socket_ptr tcp_socket_ptr,
                  const broadcaster_directory& directory,
                  size_t max_total_size,
                  time_duration max_queue_delay)
    : io_service_(io_service)
    , strand_(strand)
    , tcp_socket_ptr_(tcp_socket_ptr)
    , directory_(directory)
    , max_total_size_(max_total_size)
    , max_queue_delay_(max_queue_delay)
    , max_delay_(boost::posix_time::seconds(0))
    , preamble_without_header_("")
    , last_tick_time_(boost::posix_time::microsec_clock::universal_time())
    , status_(status_init)
    , start_async_write_(false) {
    async_receive_command();
  }
  
  ~data_connection() {
    close();
  }
  
  std::string stream_name() const {
    if (status_ == status_configured) {
      std::ostringstream oss;
      BOOST_FOREACH(const boost::regex& r, stream_names_)
        oss << r << " ";
      return oss.str();    
    }
    return ((status_ == status_init) ? "[INIT]" : "[ERROR]");
  }
  
  static sptr make(boost::asio::io_service& io_service,
                   boost::asio::strand& strand,
                   tcp_socket_ptr p,
                   const broadcaster_directory& directory,
                   size_t max_total_size=40*1024*1024,
                   time_duration max_queue_delay=boost::posix_time::minutes(5)) {
    return sptr(new data_connection(io_service, strand, p, directory, max_total_size, max_queue_delay));
  }
  
  // close() must ONLY be called in the destructor
  // for all other purposes set status_= status_error 
  // then the object connection will be deleted in the next call to broadcaster::push_back
  void close() {
    if (is_open()) {
      boost::system::error_code ec;      
      LOG_INFO(str(boost::format("data_connection::close ep=%s") % tcp_socket_ptr_->remote_endpoint(ec)));
      LOG_INFO("close and shutdown socket");
      tcp_socket_ptr_->shutdown(boost::asio::ip::tcp::socket::shutdown_send, ec);
      if (ec) LOG_WARNING((str(boost::format("shutdown error_code=%s") % ec)));
      tcp_socket_ptr_->close();
    }
  }
  bool is_open() const { return tcp_socket_ptr_ ? tcp_socket_ptr_->is_open() : false; }

  void pop_front() { list_of_packets_.pop_front(); }
  void pop_back()  { list_of_packets_.pop_back(); }

  list_of_packets::const_reference front() const { return list_of_packets_.front(); }
  list_of_packets::const_reference back() const { return list_of_packets_.back(); }

  bool   empty() const { return list_of_packets_.empty(); }
  size_t size()  const { return list_of_packets_.size(); }

  size_t total_size() const {
    size_t sum(0);
    BOOST_FOREACH(const list_of_packets::value_type& lp, list_of_packets_)
      sum += lp.second->size();
    return sum;
  }

  boost::asio::ip::tcp::endpoint local_endpoint(boost::system::error_code& ec) const {
    return tcp_socket_ptr_->local_endpoint(ec);
  }
  boost::asio::ip::tcp::endpoint remote_endpoint(boost::system::error_code& ec) const {
    return tcp_socket_ptr_->remote_endpoint(ec);
  }

  time_duration delay(ptime t) const {
    return (empty()) ? boost::posix_time::seconds(0) : t - front().first;
  }

  time_duration max_delay() const { return max_delay_; }
  void reset_max_delay() { max_delay_ = boost::posix_time::seconds(0); }
  double max_delay_msec() const {
    return 1e3*max_delay().ticks() / time_duration::ticks_per_second();
  }
  
  ptime last_tick_time() const { return last_tick_time_; }

  bool match_path(std::string path) const {
    BOOST_FOREACH(const boost::regex& r, stream_names_)
      if (regex_match(path, r)) return true;
    return false;
  }
  
  bool check_format(data_ptr d) const {
    if (not d) return true;
    const header* hp(reinterpret_cast<const header*>(&(*d)[0]));
    return (sizeof(header)+hp->length() == d->size());
  }
  data_type strip_header(data_ptr d) const {
    assert(d->size() >= sizeof(header));
    return data_type(d->begin()+sizeof(header), d->end());
  }

  bool push_back(ptime t,
                 std::string path,
                 data_ptr data,
                 data_ptr preamble) {
    if (status_ == status_init)  return true;
    if (status_ == status_error) return false;
    // path="" is always broadcasted
    if (path != "" && !match_path(path))
      return true;

    // pedantic checks
    assert(check_format(data) == true);
    assert(check_format(preamble) == true);

    max_delay_ = std::max(max_delay_, delay(t));

    // if the buffer is full forget all data except first packets which may be being sent
    size_t n_omit(0);
    if (not empty() && (total_size() > max_total_size_ || front().first+max_queue_delay_ < t)) {
      for (; size()>1; ++n_omit)      
        list_of_packets_.pop_back();
    }
    if (n_omit != 0)
      LOG_WARNING(str(boost::format("omitted # %d data packets") % n_omit));

    if (is_open()) {
      const bool list_was_empty(empty());
      if (preamble) {
        const data_type preamble_without_header(strip_header(preamble));
        if (preamble_without_header_ != preamble_without_header) {
          preamble_without_header_= preamble_without_header;
          list_of_packets_.push_back(std::make_pair(t, preamble));
        }
      }
      list_of_packets_.push_back(std::make_pair(t, data));
      if (list_was_empty || start_async_write_) {
        start_async_write_= false;
        async_write_data();
      }
      return true;
    }
    return false;
  }

  void async_write_data() {
    if (!empty() && is_open()) {
      data_ptr dataPtr(front().second);
      const header* hp((const header*)dataPtr->data());
      if (hp->length() != dataPtr->size() - sizeof(header)) {
        LOG_ERROR(str(boost::format("ERROR: %d!=%d (%s)") % hp->length() % (dataPtr->size() - sizeof(header)) % (*hp)));
      }
      boost::asio::async_write(*tcp_socket_ptr_,
                               boost::asio::buffer(dataPtr->data(), dataPtr->size()),
                               strand_.wrap(boost::bind(&data_connection::handle_write_data,
                                                        this,
                                                        boost::asio::placeholders::error,
                                                        boost::asio::placeholders::bytes_transferred)));
    }
  }

  void handle_write_data(const boost::system::error_code& ec,
                         std::size_t bytes_transferred) {
    if (ec) {
      LOG_WARNING(str(boost::format("handle_write_data ec=%s %d") % ec.message() % bytes_transferred));
      status_= status_error; // -> close();
    } else if (is_open()) {
      pop_front();
      async_write_data();
    }
  }

  void async_receive_command() {
    LOG_INFO(str(boost::format("async_receive_command status=%d") % status_));
    switch (status_) {
    case status_init:
      boost::asio::async_read_until(*tcp_socket_ptr_, response_, "\r\n",
                                    strand_.wrap(boost::bind(&data_connection::handle_receive_command,
                                                             this,
                                                             boost::asio::placeholders::error)));
      break;
    case status_configured:
      break;
    default:
      // this should never happen
      ;
    }
  }

  void handle_receive_command(const boost::system::error_code& ec) {
    if (ec) {
      LOG_INFO(str(boost::format("handle_receive_command ec=%s") % ec));
      status_= status_error; // -> close();
      return;
    }
    std::istream response_stream(&response_);
    std::string action;
    if (response_stream >> action) {
      LOG_INFO(str(boost::format("handle_receive_command action='%s'") % action));
      if (action == "LIST") {
        // send list of available streams        
        send_reply(directory_.ls(), status_);
      } else if (action == "GET") {
        std::string stream_name;
        while (response_stream >> stream_name) {          
          LOG_INFO(str(boost::format("handle_receive_command requested stream name='%s'") % stream_name));
          boost::regex stream_regex;
          try { // stream_name may not be a proper reguar expression
            stream_regex = boost::regex(stream_name);
          } catch (const boost::regex_error& e) {
            LOG_ERROR(str(boost::format("handle_receive_command: invalid regex '%s'") % stream_name));
            send_reply(str(boost::format("ERROR: invalid regex '%s'") % stream_name), status_);
            return;
          }
          if (directory_.contains(stream_regex)) {
            stream_names_.insert(stream_regex);
            LOG_INFO(str(boost::format("handle_receive_command successfully requested stream name='%s'")
                         % stream_name));
          } else { // stream not available
            LOG_ERROR(str(boost::format("handle_receive_command: stream '%s' is not available") % stream_name));
            send_reply(str(boost::format("ERROR stream '%s' is not available aborting") % stream_name), status_);
            break;
          }
        }
        if (stream_names_.empty()) {
          LOG_ERROR("handle_receive_command: no stream name given");
          send_reply("ERROR: no stream name given", status_);
          return;
        }
        send_reply("OK", status_configured);
      } else { // action != {LIST,GET}
        LOG_ERROR("handle_receive_command: action != {LIST,GET}");
        send_reply("ERROR: action != {LIST,GET}", status_);
      }
    } else { // no action
      LOG_ERROR("handle_receive_command: no action");
      // send_reply("ERROR: no action", status_);
      async_receive_command();
    }
  }

  void send_reply(std::string reply, status_type new_status) {
    reply_= reply+"\r\n";
    boost::asio::async_write(*tcp_socket_ptr_,
                             boost::asio::buffer(reply_),
                             strand_.wrap(boost::bind(&data_connection::handle_write_reply,
                                                      this,
                                                      new_status,
                                                      boost::asio::placeholders::error,
                                                      boost::asio::placeholders::bytes_transferred)));
  }

  void handle_write_reply(status_type new_status,
                          boost::system::error_code ec,
                          std::size_t bytes_transferred) {
    LOG_INFO(str(boost::format("handle_write_reply: ec='%s' bytes_transferred=%d new_status=%d")
                 % ec % bytes_transferred % new_status));
    if (ec) {
      async_receive_command();
    } else {
      // this is tricky: if we do not wait long enough
      // the client will receiver more data than expected in async_read_until:
      // therefore the status is updated only after the first tick has been received
      if (new_status == status_configured) {
        // make sure that first the current directory is broadcasted
        const ptime now(boost::posix_time::microsec_clock::universal_time());
        data_ptr dp(new data_type(directory_.serialize(now)));
        list_of_packets_.push_back(std::make_pair(now, dp));
        start_async_write_ = true; // this makes push_back start async_write with non-empty queue
        async_receive_tick(new_status);
      }
      else
        async_receive_command();
    }
  }

  void async_receive_tick(status_type status=status_configured) {
    if (is_open())
      boost::asio::async_read(*tcp_socket_ptr_,
                              boost::asio::buffer(&dummy_data_, sizeof(dummy_data_)),
                              strand_.wrap(boost::bind(&data_connection::handle_receive_tick,
                                                       this,
                                                       boost::asio::placeholders::error,
                                                       boost::asio::placeholders::bytes_transferred,
                                                       status)));
  }
  void handle_receive_tick(const boost::system::error_code& ec,
                           std::size_t bytes_transferred,
                           status_type new_status) {
    if (ec) {
      LOG_INFO(str(boost::format("handle_receive_tick ec=%s") % ec));
      status_= status_error; // -> close();
    } else {
      status_= new_status;
      last_tick_time_ = boost::posix_time::microsec_clock::universal_time();
      LOG_INFO(str(boost::format("tick %d '%c' %d") % bytes_transferred % dummy_data_ % status_));
      async_receive_tick();
    }
  }
protected:
private:
  boost::asio::io_service& io_service_;
  boost::asio::strand&     strand_;
  boost::asio::streambuf   response_;
  std::string              reply_;
  tcp_socket_ptr           tcp_socket_ptr_;
  const broadcaster_directory& directory_;
  const size_t             max_total_size_;
  const time_duration      max_queue_delay_;
  time_duration            max_delay_;
  list_of_packets          list_of_packets_;
  data_type                preamble_without_header_;
  char                     dummy_data_;
  ptime                    last_tick_time_;
  std::set<boost::regex>   stream_names_;
  status_type              status_;
  bool                     start_async_write_;
} ;

#endif //  _DATA_CONNECTION_HPP_cm111219_

