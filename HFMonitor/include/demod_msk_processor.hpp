// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _DEMOD_MSK_PROCESSOR_HPP_cm130214_
#define _DEMOD_MSK_PROCESSOR_HPP_cm130214_

#include <complex>
#include <iostream>
#include <sstream>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "demod/msk.hpp"
#include "logging.hpp"
#include "processor.hpp"

class demod_msk_processor : public processor::base_iq {
public:
  typedef boost::shared_ptr<demod_msk_processor> sptr;

  typedef boost::posix_time::time_duration time_duration;

  demod_msk_processor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , name_(config.get<std::string>("<xmlattr>.name"))
    , fc_Hz_(config.get<double>("<xmlattr>.fc_Hz"))
    , fm_Hz_(config.get<double>("<xmlattr>.fm_Hz"))
    , dwl_Hz_(config.get<double>("<xmlattr>.dwl_Hz"))
    , period_Sec_(config.get<double>("<xmlattr>.period_Sec")) {}
  
  ~demod_msk_processor() {}

  void process_iq(service::sptr sp,
                  const_iterator i0,
                  const_iterator i1) {
    std::cout << "demod_msk_processor::process_iq " << sp->stream_name()
              << " sample_rate_Hz=" <<   sp->sample_rate_Hz()
              << " center_frequency_Hz=" << sp->center_frequency_Hz()
              << " fc_Hz=" << fc_Hz_
              << " fm_Hz=" << fm_Hz_
              << std::endl;
    // set up a new filter
    if (not demod_msk_)
      demod_msk_ = demod::msk::make(sp->sample_rate_Hz(), fm_Hz_, dwl_Hz_, period_Sec_);

    for (const_iterator i(i0); i!=i1; ++i) {
      demod_msk_->process(*i);
//       const double_t carrier_phase(.5*(demod_msk_->pll_plus().theta() +
//                                        demod_msk_->pll_minus().theta()));
//       const demod::msk::complex_type x(*i * std::exp(demod::msk::complex_type(0., carrier_phase)));
//       const double ct(cos(0.5*(demod_msk_->pll_plus().theta() -
//                                demod_msk_->pll_minus().theta())));
//       const double st(cos(0.5*(demod_msk_->pll_plus().theta() -
//                                demod_msk_->pll_minus().theta())));
//       std::cout << "MSK_DEMOD: " << demod_msk_->pll_plus().theta()
//                 << " " << demod_msk_->pll_minus().theta()
//                 << " " << demod_msk_->pll_plus().f1()
//                 << " " << demod_msk_->pll_minus().f1()
//                 << " " << std::arg(*i)
//                 << " " << std::arg(x)
//                 << " " << x.real()
//                 << " " << x.imag()
//                 << std::endl;
    }
  }

protected:
private:
  const std::string name_;
  const double      fc_Hz_;     // center frequency
  const double      fm_Hz_;     // modulation frequency
  const double      dwl_Hz_;
  const double      period_Sec_;
  demod::msk::sptr  demod_msk_;
} ;

#endif // _DEMOD_MSK_PROCESSOR_HPP_cm130214_
