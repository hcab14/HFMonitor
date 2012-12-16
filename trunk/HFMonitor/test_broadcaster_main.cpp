// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <boost/property_tree/xml_parser.hpp>

#include "network/broadcaster.hpp"
#include "network/protocol.hpp"
#include "run.hpp"

class test_bc {
public:
  typedef boost::shared_ptr<test_bc> sptr;
  typedef data_connection::ptime ptime;
  typedef std::string data_type;
  
  test_bc(boost::asio::io_service& io_service,
	  const boost::property_tree::ptree& config)
    : bc_(broadcaster::make(io_service, config)) {
  }

  static sptr make(boost::asio::io_service& io_service,
		   const boost::property_tree::ptree& config) {
    return sptr(new test_bc(io_service, config));
  }

  void start() { bc_->start(); }
  void stop() { bc_->stop(); }

  void callback() {
    LOG_INFO("callback");
    last_callback_time_ = boost::posix_time::microsec_clock::universal_time();
    std::string message1("Hi There");
    std::string message2("Hello World!");
    std::string message3("Howdy?");

    header h1("TEST0001", last_callback_time_, 0, message1.size());
    std::string data1;
    std::copy(h1.begin(), h1.end(), std::back_inserter(data1));
    std::copy(message1.begin(), message1.end(), std::back_inserter(data1));

    header h2("TEST0002", last_callback_time_, 0, message2.size());
    std::string data2;
    std::copy(h2.begin(), h2.end(), std::back_inserter(data2));
    std::copy(message2.begin(), message2.end(), std::back_inserter(data2));

    header h3("TEST0003", last_callback_time_, 0, message3.size());
    std::string data3;
    std::copy(h3.begin(), h3.end(), std::back_inserter(data3));
    std::copy(message3.begin(), message3.end(), std::back_inserter(data3));

    bc_->bc_data(last_callback_time_, "data_00001", data1);
    bc_->bc_data(last_callback_time_, "data_00002", data2);
    bc_->bc_data(last_callback_time_, "data_00003", data3);
  }

protected:
private:
  broadcaster::sptr bc_;
  ptime last_callback_time_;
} ;

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_broadcaster");
  try {
    const std::string filename((argc > 1 ) ? argv[1] : "config.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config);
    boost::asio::io_service io_service;

    test_bc::sptr bp = test_bc::make(io_service, config);
    bp->start();
    const size_t n(100);
    for (size_t i=0; i<n; ++i) {
      bp->callback();
      usleep(1000*1000);
    }
    bp->stop();

  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
