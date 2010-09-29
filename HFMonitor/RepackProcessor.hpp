// -*- C++ -*-
// $Id$
#ifndef _REPACK_PROCESSOR_HPP_cm100818_
#define _REPACK_PROCESSOR_HPP_cm100818_

#include <iostream>
#include <vector>
#include <complex>
#include <boost/property_tree/ptree.hpp>
#include <boost/integer.hpp>
#include "protocol.hpp"

template<typename PROCESSOR>
class RepackProcessor {
public:
  typedef typename std::vector<std::complex<double> > Buffer;
  RepackProcessor(const boost::property_tree::ptree& config)
    : p_(config)
    , bufferLengthSec_(config.get<double>("Repack.BufferLengthSec"))
    , currentHeader_()
    , buffer_(0)
    , i_(buffer_.begin()) {}

  ~RepackProcessor() {}

  void procIQ(const Header& header, const std::vector<std::complex<double> >& samples) { 
    // std::cout << "RepackProcessor::procIQ " << header << std::endl;
    if (!currentHeader_.hasEqualParameters(header))
      update(header);

    for (typename Buffer::const_iterator j=samples.begin(); j!=samples.end(); ++j) {
      if (i_ == buffer_.end()) {
	p_.procIQ(currentHeader_, buffer_);
	i_ = buffer_.begin();
      }
      if (i_ == buffer_.end()) 
	throw 1;
      else {
	*i_++ = *j;
	currentHeader_.sampleNumber()++;
      }
    }
  }

protected:
  void update(const Header& h) {
    if (h.sampleRate() == 0) 
      throw 1;
    buffer_.resize(size_t(0.5 + double(h.sampleRate()) * double(bufferLengthSec_)));
    i_ = buffer_.begin();
    currentHeader_ = h;
    // currentHeader_.subtractFromSampleNumber(buffer_.size());
    currentHeader_.sampleNumber() -= buffer_.size();
    currentHeader_.setNumberOfSamples(buffer_.size());
  }

private:
  PROCESSOR p_;
  double bufferLengthSec_;
  Header currentHeader_;
  Buffer buffer_;
  Buffer::iterator i_;
} ;

#endif // _REPACK_PROCESSOR_HPP_cm100818_

