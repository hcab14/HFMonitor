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

#include <iostream>
#include <sstream>
#include <boost/filesystem.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include "network.hpp"
#include "network/client/client_multi_base.hpp"
#include "run.hpp"

int main(int argc, char* argv[])
{
  boost::program_options::variables_map vm;
  try {
    vm = process_options("config/multi_downconvert.xml", argc, argv);
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  boost::filesystem::path p(vm["config"].as<std::string>());
  LOGGER_INIT("./Log", p.stem().native());
  try {
    boost::property_tree::ptree config;
    read_xml(vm["config"].as<std::string>(), config,
             boost::property_tree::xml_parser::no_comments);

    boost::asio::signal_set signals
      (network::get_io_service(), SIGINT, SIGTERM, SIGQUIT);
    signals.async_wait
      (boost::bind
       (&boost::asio::io_service::stop, boost::ref(network::get_io_service())));

    network::client::client_multi_base c(network::get_io_service(), config);
    c.start();

    boost::thread_group threadpool;
    for (size_t i(0); i<4; ++i)
      threadpool.create_thread
        (boost::bind
         (&boost::asio::io_service::run,
          boost::ref(network::get_io_service())));

    threadpool.join_all();
  } catch (const std::exception &e) {
    LOG_ERROR(e.what());
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
