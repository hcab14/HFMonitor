// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _REPACK_PROCESSOR_HPP_cm100818_
#define _REPACK_PROCESSOR_HPP_cm100818_

#include <iostream>
#include <vector>
#include <complex>
#include <boost/integer.hpp>
#include <boost/property_tree/ptree.hpp>

#include "logging.hpp"
#include "network.hpp"
#include "processor.hpp"
#include "processor/IQBuffer.hpp"

template<typename PROCESSOR>
class repack_processor : public processor::base_iq {
public:
  repack_processor(boost::asio::io_service&           service,
                   const boost::property_tree::ptree& config)
    : base_iq(config)
    , p_(service, config)
    , bufferLengthSec_(config.get<double>("Repack.<xmlattr>.bufferLength_sec"))
    , overlap_(   0.01*config.get<double>("Repack.<xmlattr>.overlap_percent"))
    , service_()
    , iqBuffer_(4, 0.0)
    , counter_(0) {}

  repack_processor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , p_(config)
    , bufferLengthSec_(config.get<double>("Repack.<xmlattr>.bufferLength_sec"))
    , overlap_(   0.01*config.get<double>("Repack.<xmlattr>.overlap_percent"))
    , service_()
    , iqBuffer_(4, 0.0)
    , counter_(0) {}

  ~repack_processor() {}

  boost::asio::io_service& get_service() { return service_; }

  void process_iq(processor::base_iq::service::sptr sp,
                  processor::base_iq::const_iterator i0,
                  processor::base_iq::const_iterator i1) {
    using namespace boost::posix_time;
    if (!service_)
      iqBuffer_.update(size_t(sp->sample_rate_Hz()*bufferLengthSec_), overlap_);
    
    service_ = sp;
    const time_duration dt
      (0,0,0, counter_*time_duration::ticks_per_second()/service_->sample_rate_Hz());
    service_->update_ptime(-dt);

    // if (!currentHeader_.hasEqualParameters(header)) 
    //   update(header);    

    // keep track of sample numbers
    for (; i0 != i1; ++i0) {
      ++counter_;
      iqBuffer_.insert(this, *i0);
    }
  }
  
  // called from IQBuffer::insert 
  void procIQ(std::vector<std::complex<double> >::const_iterator i0,
              std::vector<std::complex<double> >::const_iterator i1) {
    // std::cout << "counter= " << counter_ << " " << iqBuffer_.m() << std::endl;
    if (counter_ == iqBuffer_.m());
    p_.process_iq(service_, i0, i1);
    counter_ = 0;
  }

protected:
  // void update(const Header& h) {
  //   ASSERT_THROW(h.sampleRate() != 0);
  //   iqBuffer_.update(size_t(h.sampleRate()*bufferLengthSec_), overlap_);
  //   currentHeader_ = h;
  //   currentHeader_.sampleNumber() -= iqBuffer_.n();
  //   currentHeader_.setNumberOfSamples(iqBuffer_.n());
  //   counter_ = 0;
  // }

private:
  PROCESSOR p_;
  double    bufferLengthSec_;
  double    overlap_;
  processor::base_iq::service::sptr service_;
  IQBuffer  iqBuffer_;
  boost::int64_t counter_;
} ;

#endif // _REPACK_PROCESSOR_HPP_cm100818_

