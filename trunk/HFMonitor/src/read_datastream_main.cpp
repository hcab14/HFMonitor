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
#include <string>
#include <vector>

#include "network.hpp"
#include "network/broadcaster/directory.hpp"
#include "network/client/service_net.hpp"
#include "network/protocol.hpp"
#include "network/iq_adapter.hpp"
#include "repack_processor.hpp"

#include "FFTProcessor.hpp"
#include "demod_msk_processor.hpp"
#include "processor/registry.hpp"
#include "network/iq_adapter.hpp"

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/xml_parser.hpp>


namespace detail {
  template<typename T>
  T readT(std::istream& is) {
    T data;
    is.read((char *)(&data), sizeof(T));
    if (!is) throw std::runtime_error("read failed");
    return data;
  }
}

class read_datastream {
public:
  read_datastream(processor::base::sptr pp)
    : processor_(pp) {}
  ~read_datastream() {}

  class service_read_data_stream : public service_net {
  public:
    typedef boost::shared_ptr<service_read_data_stream> sptr;
    virtual ~service_read_data_stream() {}

    static sptr make(const header& h, const broadcaster_directory& d) {
      return sptr(new service_read_data_stream(h, d));
    }

    virtual void put_result(processor::result_base::sptr r) {
      std::cout << r->name() << " " << make_time_label(r->approx_ptime()) << " ";
      r->dump_data(std::cout) << std::endl;
    }
    
  protected:
    service_read_data_stream(const header& h, const broadcaster_directory& d)
      : service_net(h, d) {}

    std::string make_time_label(ptime t) const {
      std::stringstream oss;
      oss.imbue(std::locale(oss.getloc(), new boost::posix_time::time_facet("%Y-%m-%d %H:%M:%s")));
      oss << t;
      return oss.str();
    }
  private:
    service_read_data_stream(const service_read_data_stream& );
    service_read_data_stream& operator=(const service_read_data_stream& );
  } ;

  bool process_file(std::string filename) {
    std::ifstream ifs(filename.c_str(), std::ios::binary);
    while (ifs) {
      try {
	header_ = detail::readT<header>(ifs);
	if (header_.length()) {
	  std::string bytes(" ", header_.length());
	  ifs.read(&bytes[0], header_.length());
	  if (header_.id() == directory_.id()) {
	    directory_.update(header_.length(), &bytes[0]);
	  } else {
	    if (processor_) {
	      // make up service object
	      processor::service_base::sptr sp(service_read_data_stream::make(get_header(), get_directory()));
	      processor_->process(sp, &bytes[0], &bytes[0]+header_.length());
	    }
	  }
	} else {
	  return false;
	}
      } catch (const std::exception &e) {
	// std::cout << "AAA " << e.what() << std::endl;
	return false;
      }
    }
    return true;
  }

protected:
  const broadcaster_directory& get_directory() const { return directory_; }
  const header& get_header() const { return header_; }

private:
  broadcaster_directory directory_;
  header                header_;
  processor::base::sptr processor_;
} ;

int main(int argc, char* argv[])
{
  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,?",                                           "produce help message")
    ("version,v",                                        "display version")
    ("config,c", po::value<std::string>(),               "path to XML configuration file")
    ("key,k",    po::value<std::string>(),               "key in config file defining the processor to be used")
    ("input,i",  po::value<std::vector<std::string> >(), "input files");
  po::positional_options_description p;
  p.add("input", -1);
  
  po::variables_map vm;
  try {
    
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    po::notify(vm);

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
  
  LOGGER_INIT("./Log", "read_datastream");

  try {
    // make up ptree config
    boost::property_tree::ptree config;
    read_xml(vm["config"].as<std::string>(), config, boost::property_tree::xml_parser::no_comments);

    processor::registry::add<iq_adapter<FFTProcessor<float > > >("FFTProcessor_FLOAT");
    processor::registry::add<iq_adapter<FFTProcessor<double> > >("FFTProcessor_DOUBLE");
    processor::registry::add<iq_adapter<demod_msk_processor  > >("DemodMSK");

    const boost::property_tree::ptree& processor_config(config.get_child(vm["key"].as<std::string>()));
    
    std::cout << "type= " << processor_config.get<std::string>("<xmlattr>.type") << std::endl;
    read_datastream rds(processor::registry::make(processor_config.get<std::string>("<xmlattr>.type"),
						  processor_config));

    if (vm.count("input")) {
      std::cout << "input: [" << vm.count("input") << "]: ";
      BOOST_FOREACH(const std::string& input_filename,  vm["input"].as<std::vector<std::string> >()) {
	std::cout << input_filename << std::endl;
	rds.process_file(input_filename);
      }
      return 1;
    }
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
