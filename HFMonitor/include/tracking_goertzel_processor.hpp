// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _TRACKING_GOERTZEL_PROCESSOR_HPP_cm130211_
#define _TRACKING_GOERTZEL_PROCESSOR_HPP_cm130211_

#include <iostream>
#include <vector>
#include <complex>
#include <boost/property_tree/ptree.hpp>

#include "filter/tracking_goertzel_filter.hpp"
#include "logging.hpp"
#include "processor.hpp"

class tracking_goertzel_processor : public processor::base_iq {
public:
  typedef boost::shared_ptr<tracking_goertzel_processor> sptr;

  tracking_goertzel_processor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , f0_Hz_(config.get<double>("<xmlattr>.F0_Hz"))
    , min_df_Hz_(config.get<double>("<xmlattr>.MinDf_Hz"))
    , max_history_size_(config.get<size_t>("<xmlattr>.MaxHistorySize")) {}
  
  ~tracking_goertzel_processor() {}

  void process_iq(service::sptr sp,
                  const_iterator i0,
                  const_iterator i1) {
    // set up a new filter
    const bool update_filter(filter_ ? (filter_->fs_Hz() != sp->sample_rate_Hz()) : true);
    if (update_filter)
      filter_ = tracking_goertzel_filter::make(sp->sample_rate_Hz(),
                                               f0_Hz_-sp->center_frequency_Hz(),
                                               min_df_Hz_,
                                               max_history_size_);
    for (; i0!=i1; ++i0)
      filter_->update(*i0);
  }
  
protected:
private:
  const double f0_Hz_;
  const double min_df_Hz_;
  const size_t max_history_size_;
  tracking_goertzel_filter::sptr filter_;
} ;

#endif // _TRACKING_GOERTZEL_PROCESSOR_HPP_cm130211_

