// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

#include <iostream>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

#include "network.hpp"
#include "network/client/client_base.hpp"

int main(int argc, char* argv[])
{
  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,?",                                                       "produce help message")
    ("host,h", po::value<std::string>()->default_value("127.0.0.1"), "server hostname")
    ("port,p", po::value<std::string>()->default_value("18001"),     "server port");

  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 1;
    }
  } catch (const std::exception &e) {
    std::cout << e.what() << std::endl;
    std::cout << desc << std::endl;
    return 1;
  }

  LOGGER_INIT("./Log", "server_ls");
  try {
    // make up ptree config
    boost::property_tree::ptree config;
    config.put("server.<xmlattr>.host", vm["host"].as<std::string>());
    config.put("server.<xmlattr>.port", vm["port"].as<std::string>());

    client_base c(network::get_io_service(), config);

    const std::set<std::string> streams(c.ls());
    BOOST_FOREACH (std::string s, streams) {
      std::cout << s;
    }
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
