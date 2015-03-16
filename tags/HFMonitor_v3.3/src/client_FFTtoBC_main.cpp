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
#include <boost/filesystem.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "FFTProcessorToBC.hpp"
#include "network/client.hpp"
#include "network/iq_adapter.hpp"
#include "processor/registry.hpp"
#include "repack_processor.hpp"
#include "run.hpp"

/*! \addtogroup executables
 *  @{
 * \addtogroup client_FFTtoBC client_FFTtoBC
 * client_FFTtoBC_main
 * - @ref FFTProcessor with output to @ref network::broadcaster::broadcaster
 * - configuration using command-line / XML
 * 
 * @{
 */
int main(int argc, char* argv[])
{
  // set up command-line options
  boost::program_options::variables_map vm;
  try {
    vm = process_options("config/FFTProcessor.xml", argc, argv);
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  boost::filesystem::path p(vm["config"].as<std::string>());
  LOGGER_INIT("./Log", p.stem().native());
  try {
    // read the XML initialization file
    boost::property_tree::ptree config;
    read_xml(vm["config"].as<std::string>(), config, boost::property_tree::xml_parser::no_comments);

    // register FFT processors
    processor::registry::add<FFTProcessorToBC<float>  >("FFTProcessorToBC_FLOAT");
    processor::registry::add<FFTProcessorToBC<double> >("FFTProcessorToBC_DOUBLE");

    // connect to DataIQ stream
    const std::string stream_name("DataIQ");
    network::client::client<network::iq_adapter<repack_processor<FFTProcessorToBC<double> > > >
      c(config.get_child("FFTProcessor"));

    const std::set<std::string> streams(c.ls());
    if (streams.find(stream_name) != streams.end())
      ASSERT_THROW(c.connect_to(stream_name) == true);
    else
      throw std::runtime_error(str(boost::format("stream '%s' is not available")
                                   % stream_name));
    // start
    c.start();

    // run until a signal is caught
    run_in_thread(network::get_io_service());
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
/// @}
/// @}

