// -*- C++ -*-
// $Id$
//
// Copyright 2010-2013 Christoph Mayer
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

#include <complex>
#include <iostream>

#include <boost/property_tree/xml_parser.hpp>

#include "gen_filename.hpp"
#include "network.hpp"
#include "network/client.hpp"
#include "network/iq_adapter.hpp"
#include "repack_processor.hpp"
#include "run.hpp"

#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <iostream>
#include <vector>
#include <cmath>


class save_datastream : public processor::base, public gen_filename {
public:
  save_datastream(const boost::property_tree::ptree& config)
    : base(config) 
    , base_path_(config.get<std::string>("<xmlattr>.filePath"))
    , file_period_(gen_filename::str2period(config.get<std::string>("<xmlattr>.filePeriod")))
    , pos_(0)
    , stream_number_(0) {
  }
  virtual ~save_datastream() {}

  virtual std::string fileExtension() const { return "bin"; }
  virtual file_period filePeriod() const { return file_period_; }

  virtual void process(service::sptr sp,
		       const_iterator begin,
		       const_iterator end) {
#if 1
    const size_t length(std::distance(begin, end));
    std::cout << "process " << sp->id()
              << " " << sp->approx_ptime()
              << " " << sp->stream_name()
	      << " length= " << length
              << std::endl;
#endif
    const boost::filesystem::path
      filepath(gen_file_path(base_path_, sp->stream_name(), sp->approx_ptime()));
    if (boost::filesystem::exists(filepath) and (pos_ == std::streampos(0))) {
      std::cerr << "file '" << filepath << "' exists and will be overwritten" << std::endl;
      boost::filesystem::remove(filepath);
    }
    if (not boost::filesystem::exists(filepath)) {
      std::ofstream ofs(filepath.c_str(), std::ios::binary);        
      if (not directory_.contains(sp->stream_name())) {
	const std::pair<boost::uint32_t, bool> r(directory_.insert(sp->stream_name()));
	ASSERT_THROW(r.second == true);
	stream_number_ = r.first;
      }

      const std::string sd(directory_.serialize(sp->approx_ptime()));
      const network::protocol::header h(directory_.id(), sp->approx_ptime(), 0, sd.size());
      std::copy(h.begin(),  h.end(),  std::ostream_iterator<char>(ofs, ""));
      std::copy(sd.begin(), sd.end(), std::ostream_iterator<char>(ofs, ""));
      pos_ = ofs.tellp();
    }
    std::ofstream ofs(filepath.c_str(), std::ios::in | std::ios::out | std::ios::binary);
    ofs.seekp(pos_);
    const network::protocol::header h
      (sp->id(), sp->approx_ptime(), stream_number_, std::distance(begin, end));
    std::copy(h.begin(), h.end(), std::ostream_iterator<char>(ofs, ""));
    std::copy(begin,     end,     std::ostream_iterator<char>(ofs, ""));
    pos_ = ofs.tellp();      
  }
private:
  std::string                     base_path_;
  gen_filename::file_period       file_period_;
  std::streampos                  pos_;    
  network::broadcaster::directory directory_;
  boost::uint32_t                 stream_number_;
} ;

int main(int argc, char* argv[])
{
  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,?",                                                         "produce help message")
    ("version,v",                                                      "display version")
    ("host,h",   po::value<std::string>()->default_value("127.0.0.1"), "server hostname")
    ("port,p",   po::value<std::string>()->default_value("18001"),     "server port")
    ("stream,s", po::value<std::string>()->default_value("DataIQ"),    "stream name")
    ("path",     po::value<std::string>()->default_value("Data"),      "file path for saved data")
    ("period",   po::value<std::string>()->default_value("5m"),        "file period (1m,5m,1h,1d)");

  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 1;
    }
    if (vm.count("version")) {
      std::cout << SVN_VERSION_STRING << std::endl;
      return 1;
    }
  } catch (const std::exception &e) {
    std::cout << e.what() << std::endl;
    std::cout << desc << std::endl;
    return 1;
  }
  
  LOGGER_INIT("./Log", "save_datastream");

  try {
    // make up ptree config
    boost::property_tree::ptree config;
    config.put("server.<xmlattr>.host", vm["host"].as<std::string>());
    config.put("server.<xmlattr>.port", vm["port"].as<std::string>());
    config.put("<xmlattr>.filePath",    vm["path"].as<std::string>());
    config.put("<xmlattr>.filePeriod",  vm["period"].as<std::string>());

    const std::string stream_name(vm["stream"].as<std::string>());
    network::client::client<save_datastream> c(config);

    const std::set<std::string> streams(c.ls());
    if (streams.find(stream_name) != streams.end())
      ASSERT_THROW(c.connect_to(stream_name) == true);
    else
      throw std::runtime_error(str(boost::format("stream '%s' is not available")
                                   % stream_name));
    c.start();
    run_in_thread(network::get_io_service());
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
