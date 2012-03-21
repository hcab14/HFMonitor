// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _DATA_CONNECTION_HPP_cm111219_
#define _DATA_CONNECTION_HPP_cm111219_

#include <iostream>
#include <iterator>
#include <string>
#include <deque>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/foreach.hpp>

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
    status_init       = 0,  // negotiation phase: client asks for a certain stream
    status_configured = 1   // after a certain stream has been selected, data is sent to the client
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
    , last_tick_time_(boost::posix_time::microsec_clock::universal_time())
    , stream_name_("")
    , status_(status_init) {
    for (broadcaster_directory::const_iterator i(directory_.begin());
         i != directory_.end(); ++i)
      std::cout << "dir: '" << *i << "'" << std::endl;
    async_receive_command();
  }

  ~data_connection() {
    close();
  }

  broadcaster_directory::index_type stream_name() const {
    return (status_ == status_configured ? stream_name_ : "[INIT]");
  }

  static sptr make(boost::asio::io_service& io_service,
                   boost::asio::strand& strand,
                   tcp_socket_ptr p,
                   const broadcaster_directory& directory,
                   size_t max_total_size=40*1024*1024,
                   time_duration max_queue_delay=boost::posix_time::minutes(5)) {
    return sptr(new data_connection(io_service, strand, p, directory, max_total_size, max_queue_delay));
  }

  void close() {
    if (is_open()) {
      boost::system::error_code ec;      
      LOG_INFO(str(boost::format("data_connection::close ep=%s") % tcp_socket_ptr_->remote_endpoint(ec)));
      LOG_INFO("close and shutdown socket");
      if (tcp_socket_ptr_->is_open()) {
        tcp_socket_ptr_->shutdown(boost::asio::ip::tcp::socket::shutdown_send, ec);
        if (ec) LOG_WARNING((str(boost::format("shutdown error_code=%s") % ec)));
        tcp_socket_ptr_->close();
      }
    }
  }
  bool is_open() const { return tcp_socket_ptr_->is_open(); }

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

  bool push_back(ptime t, std::string path, const data_ptr& dp) {
    if (status_ != status_configured || stream_name_ != path)
      return true;

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
      const bool listOfPacketsWasEmpty(empty());
      list_of_packets_.push_back(std::make_pair(t, dp));
      if (listOfPacketsWasEmpty)
        async_write_data();
      return true;
    }
    return false;
  }

  void async_write_data() {
    if (!empty() && is_open()) {
      data_ptr dataPtr(front().second);
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
      close();
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
    std::istream response_stream(&response_);
    std::string action;
    if (response_stream >> action) {
      LOG_INFO(str(boost::format("handle_receive_command action='%s'") % action));
      if (action == "LIST") {
        // send list of available streams        
        send_reply(directory_.ls(), status_);
      } else if (action == "GET") {
        std::string stream_name;
        if (response_stream >> stream_name) {
          LOG_INFO(str(boost::format("handle_receive_command requested stream name='%s'") % stream_name));
          if (directory_.contains(stream_name)) {
            stream_name_ = stream_name;
            LOG_INFO(str(boost::format("handle_receive_command successfully requested stream name='%s'")
                         % stream_name));
            send_reply("OK", status_configured);
          } else { // stream not available
            LOG_ERROR(str(boost::format("handle_receive_command: stream '%s' is not available") % stream_name));
            send_reply(str(boost::format("ERROR stream '%s' is not available") % stream_name), status_);                                                              
          }
        } else { // no stream name
          LOG_ERROR("handle_receive_command: no stream name given");
          send_reply("ERROR: no stream name given", status_);
        }
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
      status_= new_status;
      if (status_ != status_configured)
        async_receive_command();
    }
  }

  void async_receive_tick() {
    if (is_open())
      boost::asio::async_read(*tcp_socket_ptr_,
                              boost::asio::buffer(&dummy_data_, sizeof(dummy_data_)),
                              strand_.wrap(boost::bind(&data_connection::handle_receive_tick,
                                                       this,
                                                       boost::asio::placeholders::error,
                                                       boost::asio::placeholders::bytes_transferred)));
  }
  void handle_receive_tick(const boost::system::error_code& ec,
                           std::size_t bytes_transferred) {
    if (ec) {
      LOG_INFO(str(boost::format("handle_receive_tick ec=%s") % ec));
      close();
    } else {
      last_tick_time_ = boost::posix_time::microsec_clock::universal_time();
      LOG_INFO(str(boost::format("tick %d '%c'") % bytes_transferred % dummy_data_));
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
  char                     dummy_data_;
  ptime                    last_tick_time_;
  broadcaster_directory::index_type stream_name_;
  status_type              status_;
} ;

#endif //  _DATA_CONNECTION_HPP_cm111219_

