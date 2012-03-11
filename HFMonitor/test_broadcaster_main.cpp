// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <boost/property_tree/xml_parser.hpp>
#include "broadcaster.hpp"

class test_bc : public broadcaster {
public:
  typedef boost::shared_ptr<test_bc> sptr;
  static sptr make(boost::asio::io_service& io_service,
		   const boost::property_tree::ptree& config) {
    return sptr(new test_bc(io_service, config));
  }

  void callback() {
    LOG_INFO("callback");
    last_callback_time_ = boost::posix_time::microsec_clock::universal_time();
    std::string data="data";
    bc_data(last_callback_time_, "iq",   data);
    bc_data(last_callback_time_, "test", data);
    do_pending_io_actions();
  }

  virtual void start_data_source() {
    LOG_INFO("start_data_source");
  }

  virtual void stop_data_source() {
    LOG_INFO("stop_data_source");
  }

protected:
private:
  test_bc(boost::asio::io_service& io_service,
	  const boost::property_tree::ptree& config)
    : broadcaster(io_service, config) {}

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
    size_t n=100;
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
