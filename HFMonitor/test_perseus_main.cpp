// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "include/sdr/perseus/perseus_control.hpp"

// unsigned get_hex(const boost::property_tree::ptree& pt,
//                  std::string key, std::string def) {
//   unsigned result(0);
//   std::stringstream iss;
//   iss << std::hex << pt.get<std::string>(key, def);
//   iss >> result;
//   return result;
// }

class test_cb : public Perseus::callback {
public:
  virtual ~test_cb() {}
  void operator()(unsigned char* data, size_t length) {
    // std::cout << "cb length= " << length << std::endl;
    // std::cout << "cb END" << std::endl;
  }
protected:
private:
} ;


int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_perseus");
  try {
    libusb::session::sptr s = libusb::session::get_global_session();
    const std::string filename((argc > 1 ) ? argv[1] : "config_perseus.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config);
    const boost::property_tree::ptree& config_perseus(config.get_child("Test.Perseus"));

    const size_t num_perseus = Perseus::receiver_control::get_num_perseus();
    std::cout << "found " << num_perseus << std::endl;

    if (num_perseus == 0)
      throw std::runtime_error("no perseus receiver found");    
    
    int index(-1);
    for (size_t i=0; i<num_perseus; ++i) {
      const Perseus::product_id pid(Perseus::receiver_control::get_product_id_at(i));
      std::cout << str(boost::format("index=%2d: %s") % i % pid.to_str()) << std::endl;
      if (pid.sn() == config_perseus.get<size_t>("<xmlattr>.sn", 0))
        index = i;
    }

    if (index != -1) {
      Perseus::callback::sptr cb(new test_cb);
      Perseus::receiver_control::sptr rec(Perseus::receiver_control::make(index));
      rec->init(config_perseus);
      usleep(500*1000);
      rec->start_async_input(cb);
      sleep(1);
      rec->stop_async_input();
    }
    
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
