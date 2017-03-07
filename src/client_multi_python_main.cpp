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

#include "client_multi_python_processor.hpp"

#include <boost/filesystem.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "run.hpp"

int main(int argc, char* argv[])
{
  Py_InitializeEx(0);
  PyEval_InitThreads();
  PyEval_ReleaseLock();

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

    client_multi_python_processor c(network::get_io_service(), config);
    c.start();

    boost::asio::signal_set signals
      (network::get_io_service(), SIGINT, SIGTERM, SIGQUIT);
    signals.async_wait
      (boost::bind
       (&boost::asio::io_service::stop, boost::ref(network::get_io_service())));

    boost::thread_group& threadpool = get_thread_pool();
    const size_t thread_pool_size(config.get<size_t>("ClientMulti.<xmlattr>.threadPoolSize", 4));
    for (size_t i(0); i<thread_pool_size; ++i)
      threadpool.create_thread
        (boost::bind
         (&boost::asio::io_service::run,
          boost::ref(network::get_io_service())));

    threadpool.join_all();

  } catch (boost::python::error_already_set const &) {
    PyErr_Print();
  } catch (const std::exception &e) {
    LOG_ERROR(e.what());
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
