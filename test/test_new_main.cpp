// -*- C++ -*-
// $Id$

#include <cmath>
#include <iostream>
#include <fstream>
#include <iterator>
#include <map>
#include <numeric>
#include <boost/format.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "logging.hpp"
#include "run.hpp"
#include "wave/reader.hpp"

class test_proc : processor::base_iq  {
public:
  typedef boost::shared_ptr<test_proc> sptr;
  
  test_proc(const boost::property_tree::ptree& config)
    : base_iq(config)
  {}

  void process_iq(processor::service_iq::sptr sp,
                  processor::base_iq::const_iterator i0,
                  processor::base_iq::const_iterator i1) {
    std::cout << "process_iq nS=" << std::distance(i0, i1) 
              << " " << sp->id()
              << " " << sp->approx_ptime()
              << " " << sp->sample_rate_Hz()
              << " " << sp->center_frequency_Hz()
              << " " << sp->offset_ppb()
              << std::endl;

//     const ssize_t length(std::distance(i0, i1));
  }

protected:
private:
} ;

int main(int argc, char* argv[]) {
  LOGGER_INIT("./Log", "test_new");
  try {
    const boost::program_options::variables_map
      vm(process_options("test.xml", argc, argv));

    boost::property_tree::ptree config;
    read_xml(vm["config"].as<std::string>(), config);

    wave::reader_iq<test_proc> r(config.get_child("Test"));
    for (int i((argc == 1) ? 1 : 3); i<argc; ++i) {
      std::cout << "processing " << argv[i] << std::endl;
      r.process_file(argv[i]);
    }
    r.finish();

  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    LOG_ERROR(e.what()); 
    return 1;
  }
  return 0;

}
