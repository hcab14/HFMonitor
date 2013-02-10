// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_PROCESSOR_TO_FILE_HPP_cm130310_
#define _FFT_PROCESSOR_TO_FILE_HPP_cm130310_

#include <boost/asio.hpp>

#include "FFTProcessor.hpp"

template<typename FFTFloat>
class FFTProcessorToFile : public FFTProcessor<FFTFloat> {
public:
  FFTProcessorToFile(const boost::property_tree::ptree& config)
    : FFTProcessor<FFTFloat>(config)
    , dataPath_(config.get<std::string>("FileSink.<xmlattr>.path")) {}

  virtual ~FFTProcessorToFile() {}

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


#endif // _FFT_PROCESSOR_TO_FILE_HPP_cm130310_

