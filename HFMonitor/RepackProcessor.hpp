// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _REPACK_PROCESSOR_HPP_cm100818_
#define _REPACK_PROCESSOR_HPP_cm100818_

#include <iostream>
#include <vector>
#include <complex>
#include <boost/property_tree/ptree.hpp>
#include <boost/integer.hpp>

#include "IQBuffer.hpp"
#include "protocol.hpp"

template<typename PROCESSOR>
class RepackProcessor {
public:
  typedef typename std::vector<std::complex<double> > Buffer;
  RepackProcessor(const boost::property_tree::ptree& config)
    : p_(config)
    , bufferLengthSec_(config.get<double>("Repack.BufferLengthSec"))
    , overlap_(0.01*config.get<double>("Repack.OverlapPercent"))
    , currentHeader_()
    , iqBuffer_(4, 0.0) {}

  ~RepackProcessor() {}

  void procIQ(const Header& header,
              std::vector<std::complex<double> >::const_iterator i0,
              std::vector<std::complex<double> >::const_iterator i1) { 
    // std::cout << "RepackProcessor::procIQ " << header << std::endl;
    if (!currentHeader_.hasEqualParameters(header)) 
      update(header);    

    // ptime starts from now on
    currentHeader_.approxPTime() = header.approxPTime();

    // counter_ counts samples relative to currentHeader_.approxPTime()
    counter_ = 0;

    // keep track of sample numbers
    for (; i0 != i1; ++i0) {
      ++currentHeader_.sampleNumber();
      ++counter_;
      iqBuffer_.insert(this, *i0);
    }
  }
  
  // called from IQBuffer::insert 
  void procIQ(std::vector<std::complex<double> >::const_iterator i0,
              std::vector<std::complex<double> >::const_iterator i1) {
    using namespace boost::posix_time;
    // update ptime of currentHeader
    currentHeader_.approxPTime() += 
      time_duration(0,0,0, counter_*time_duration::ticks_per_second()/currentHeader_.sampleRate());
    p_.procIQ(currentHeader_, i0, i1); 
    // reset counter_
    counter_ = 0;
  }

protected:
  void update(const Header& h) {
    if (h.sampleRate() == 0) 
      throw 1;
    iqBuffer_.update(h.sampleRate()*bufferLengthSec_, overlap_);
    currentHeader_ = h;
    currentHeader_.sampleNumber() -= iqBuffer_.n();
    currentHeader_.setNumberOfSamples(iqBuffer_.n());
    counter_ = 0;
  }

private:
  PROCESSOR p_;
  double bufferLengthSec_;
  double overlap_;
  Header currentHeader_;
  IQBuffer iqBuffer_;
  boost::int64_t counter_;
} ;

#endif // _REPACK_PROCESSOR_HPP_cm100818_

