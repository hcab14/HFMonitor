// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

#include <iostream>
#include <boost/property_tree/xml_parser.hpp>

#include "FFTProcessor.hpp"
#include "network.hpp"
#include "network/broadcaster.hpp"
#include "network/client.hpp"
#include "network/iq_adapter.hpp"
#include "processor/registry.hpp"
#include "repack_processor.hpp"
#include "run.hpp"

template<typename FFTFloat>
class FFTProcToBC : public FFTProcessor<FFTFloat> {
public:
  FFTProcToBC(const boost::property_tree::ptree& config)
    : FFTProcessor<FFTFloat>(config)
    , broadcaster_(broadcaster::make(config.get_child("Broadcaster")))
    , started_(false) {}

  virtual ~FFTProcToBC() {}

protected:
  typedef typename FFTProcessor<FFTFloat>::ResultMap ResultMap;
  virtual void dump(const typename ResultMap::value_type& result) {
    if (!started_) {
      broadcaster_->start();
      started_= false;
    }
    const std::string path(result.second->name());
    result.second->dumpToBC(path, result.first, broadcaster_);
  }
private:
  broadcaster::sptr broadcaster_;
  bool              started_;
} ;

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_client_FFTProcToBC");
  try {
    processor::registry::add<FFTProcToBC<float>  >("FFTProcToBC_FLOAT");
    processor::registry::add<FFTProcToBC<double> >("FFTProcToBC_DOUBLE");

    const std::string filename((argc > 1 ) ? argv[1] : "config_client.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config);

    const std::string stream_name("DataIQ");

    client<iq_adapter<repack_processor<FFTProcToBC<double> > > >
      c(config.get_child("FFTProcessor"));

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
