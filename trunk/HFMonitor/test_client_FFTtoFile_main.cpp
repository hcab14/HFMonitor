// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

#include <iostream>
#include <boost/property_tree/xml_parser.hpp>

#include "FFTProcessor.hpp"
#include "network.hpp"
#include "network/client_iq.hpp"
#include "repack_processor.hpp"
#include "run.hpp"

template<typename FFTFloat>
class FFTProcToFile : public FFTProcessor<FFTFloat> {
public:
  FFTProcToFile(const boost::property_tree::ptree& config)
    : FFTProcessor<FFTFloat>(config)
    , dataPath_(config.get<std::string>("FileSink.<xmlattr>.path")) {}

  virtual ~FFTProcToFile() {}

  boost::asio::io_service& get_service() { return service_; }
  
protected:
  typedef typename FFTProcessor<FFTFloat>::ResultMap ResultMap;

  virtual void dump(const typename ResultMap::value_type& result) {
    result.second->dumpToFile(dataPath_, result.first);
  }
private:
  std::string dataPath_;
  boost::asio::io_service service_;
} ;

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_client");
  try {
    const std::string filename((argc > 1 ) ? argv[1] : "config_client.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config);

    const std::string stream_name("DataIQ");

    client_iq<repack_processor<FFTProcToFile<double> > > c(config.get_child("FFTProcessor"));
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
