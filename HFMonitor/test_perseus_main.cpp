// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include "include/sdr/perseus/perseus_control.hpp"

// unsigned get_hex(const boost::property_tree::ptree& pt,
//                  std::string key, std::string def) {
//   unsigned result(0);
//   std::stringstream iss;
//   iss << std::hex << pt.get<std::string>(key, def);
//   iss >> result;
//   return result;
// }

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_perseus");
  try {
    const std::string filename((argc > 1 ) ? argv[1] : "config_perseus.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config);
    const boost::property_tree::ptree& config_perseus(config.get_child("Test.Perseus"));

    const size_t num_perseus = Perseus::receiver_control::get_num_perseus();
    std::cout << "found " << num_perseus << std::endl;

    for (size_t i=0; i<num_perseus; ++i) {
      const Perseus::product_id pid(Perseus::receiver_control::get_product_id_at(i));
      std::cout << str(boost::format("index=% 2d %s") % i % pid.to_str()) << std::endl;
    }

    if (num_perseus == 0)
      throw std::runtime_error("no perseus receiver found");    
    
    // Perseus::receiver_control::sptr rec = Perseus::receiver_control::make(0);
    // rec->init(config_perseus);
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
