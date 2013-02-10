// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_PROCESSOR_TO_BC_HPP_cm130210_
#define _FFT_PROCESSOR_TO_BC_HPP_cm130210_

#include "FFTProcessor.hpp"
#include "network/broadcaster.hpp"

template<typename FFTFloat>
class FFTProcessorToBC : public FFTProcessor<FFTFloat> {
public:
  FFTProcessorToBC(const boost::property_tree::ptree& config)
    : FFTProcessor<FFTFloat>(config)
    , broadcaster_(broadcaster::make(config.get_child("Broadcaster")))
    , started_(false) {}

  virtual ~FFTProcessorToBC() {}

protected:
  typedef typename FFTProcessor<FFTFloat>::ResultMap ResultMap;
  virtual void dump(const typename ResultMap::value_type& result) {
    if (not started_) {
      broadcaster_->start();
      started_= true;
    }
    const std::string path(result.second->name());
    result.second->dumpToBC(path, result.first, broadcaster_);
  }
private:
  broadcaster::sptr broadcaster_;
  bool              started_;
} ;

#endif // _FFT_PROCESSOR_TO_BC_HPP_cm130210_

