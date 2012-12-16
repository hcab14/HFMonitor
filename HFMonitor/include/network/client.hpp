// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _CLIENT_HPP_cm111229_
#define _CLIENT_HPP_cm111229_

#include <iostream>
#include <set>
#include <sstream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include "logging.hpp"
#include "network/broadcaster/directory.hpp"

#include "network/protocol.hpp"

// generic base class for connecting to a broadcaster
class client : private boost::noncopyable {
public:
  enum {
    max_buffer_size = 1024*1024 // 1MB
  } ;
  typedef boost::array<char, max_buffer_size> data_buffer_type;

  enum received_data_type {
    received_header, 
    received_data
  } ;

  client(boost::asio::io_service&           io_service,
         const boost::property_tree::ptree& config)
    : io_service_(io_service)
    , strand_(io_service)
    , socket_(io_service)
    , tick_buffer_('a')
    , timer_(io_service, boost::posix_time::seconds(1)) {
    if (!connect_socket(config.get<std::string>("<xmlattr>.host"),
                        config.get<std::string>("data.<xmlattr>.port")))
      throw std::runtime_error("connect failed");
  }

  virtual ~client() {
    LOG_INFO("~client: close");
    socket_.close();
  }

  // access methods
  boost::asio::io_service& get_io_service()   { return io_service_; }
  boost::asio::strand&     get_strand()       { return strand_; }  
  const header&            get_header() const { return header_; }

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
  bool connect_to(std::string name) {
    std::istringstream iss(send_request(str(boost::format("GET %s") % name)));
    std::string f;
    if (iss >> f) {
      std::cout << "f='" << f << "'" << std::endl;
      if (f == "OK") {
        async_receive_header();
        timer_.async_wait(get_strand().wrap(boost::bind(&client::on_tick, this)));
        return true;
      } else {
        std::cerr << "response= " << f << " " << name << std::endl;
        return false;
      }
    } else {
      return false;
    }
  }
  
  // stop the client in a nice way
  void stop() { get_io_service().post(get_strand().wrap(boost::bind(&client::do_close, this))); }

  // this method is to be overwitten in a derived class
  //  * get_header() provides the header
  virtual void process(data_buffer_type::const_iterator begin,
                       data_buffer_type::const_iterator end) {
    LOG_INFO(str(boost::format("process: h='%s', length=%d")
                 % get_header()
                 % std::distance(begin,end)));
  }

protected:
private:
  void do_close() {
    LOG_INFO("do_close");
    socket_.close();
  }

  void on_tick() {
    // send tick to server
    boost::asio::async_write(socket_,
                             boost::asio::buffer(&tick_buffer_, sizeof(tick_buffer_)),
                             get_strand().wrap(boost::bind(&client::handle_write_tick,
                                                           this,
                                                           boost::asio::placeholders::error,
                                                           boost::asio::placeholders::bytes_transferred,
                                                           tick_buffer_)));
    timer_.expires_at(timer_.expires_at() + boost::posix_time::seconds(1));
    timer_.async_wait(get_strand().wrap(boost::bind(&client::on_tick, this)));
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
                            boost::asio::buffer(&header_, sizeof(header)),
                            get_strand().wrap(boost::bind(&client::on_receive,
                                                          this,
                                                          received_header,
                                                          boost::asio::placeholders::error,
                                                          boost::asio::placeholders::bytes_transferred)));
  }
  
  // asyncronuosly receive the data of a data packet
  void async_receive_data() {
    ASSERT_THROW(header_.length() < data_buffer_.size());
    boost::asio::async_read(socket_,
                            boost::asio::buffer(&data_buffer_, header_.length()),
                            get_strand().wrap(boost::bind(&client::on_receive,
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
      // std::cout << "header= " << header_ << std::endl;
      async_receive_data();
      break;
    case received_data:
      assert(bytes_transferred == header_.length());
      if (header_.id() == "DIR_0000") {
//         std::string bytes;
//         std::copy(data_buffer_.begin(), data_buffer_.begin()+header_.length(), std::back_inserter(bytes));
        directory_.update(header_.length(), data_buffer_.data());
      } else {
        // process data samples in a method overwritten in a derived class
        process(data_buffer_.begin(), data_buffer_.begin()+header_.length());
      }
      // then receive the next header
      async_receive_header();
      break;
    default:
      ; // this should never happen : complaint TBD
    }
  }

  // blocking send request, returns the response
  std::string send_request(std::string request) {
    std::cout << "send_request: " << request << std::endl;
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
    std::cout << "response: " << string_buffer.str() << std::endl;
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
      std::cout << "error(connect)= " << error << std::endl;
    }
    return (!error) || (endpoint_iterator != end);
  }

  boost::asio::io_service&          io_service_;
  boost::asio::strand               strand_;
  boost::asio::ip::tcp::socket      socket_;
  header                            header_;
  data_buffer_type                  data_buffer_;
  char                              tick_buffer_;
  boost::asio::deadline_timer       timer_;
  broadcaster_directory             directory_;
} ;

#endif // _CLIENT_HPP_cm111229_
