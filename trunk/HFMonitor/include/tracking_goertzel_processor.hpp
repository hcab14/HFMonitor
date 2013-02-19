// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2013 Christoph Mayer
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#ifndef _TRACKING_GOERTZEL_PROCESSOR_HPP_cm130211_
#define _TRACKING_GOERTZEL_PROCESSOR_HPP_cm130211_

#include <complex>
#include <iostream>
#include <sstream>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "filter/tracking_goertzel_filter.hpp"
#include "logging.hpp"
#include "processor.hpp"

class tracking_goertzel_processor : public processor::base_iq {
public:
  typedef boost::shared_ptr<tracking_goertzel_processor> sptr;

  class result : public processor::result_base {
  public:
    typedef boost::shared_ptr<result> sptr;

    virtual ~result() {}
    static sptr make(std::string name,
                     ptime       t,
                     double      f0_Hz,
                     std::string state,
                     double      period_sec,
                     const detail::value_and_error& f_Hz,
                     const detail::value_and_error& df) {
      return sptr(new result(name, t, f0_Hz, state, period_sec, f_Hz, df));
    }

    double      f0_Hz()      const { return f0_Hz_; }
    std::string state()      const { return state_; }
    double      period_sec() const { return period_sec_; }
    const detail::value_and_error& f_Hz() const { return f_Hz_; }
    const detail::value_and_error& df()   const { return df_; }

    virtual std::string to_string() const {
      std::stringstream ss; 
      ss << result_base::to_string() 
         << " f0[Hz]="    << f0_Hz()
         << " state="     << state()
         << " period[s]=" << period_sec()
         << " f[Hz]="     << f_Hz()
         << " df="        << df();
      return ss.str();
    }

  protected:
  private:
    result(std::string name,
           ptime t,
           double f0_Hz,
           std::string state,
           double period_sec,
           const detail::value_and_error& f_Hz,
           const detail::value_and_error& df)
      : result_base(name, t)
      , f0_Hz_(f0_Hz)
      , state_(state)
      , period_sec_(period_sec)
      , f_Hz_(f_Hz)
      , df_(df) {}

    const double f0_Hz_;
    const std::string state_;
    const double period_sec_;
    const detail::value_and_error f_Hz_;
    const detail::value_and_error df_;
  } ;

  typedef boost::posix_time::time_duration time_duration;

  tracking_goertzel_processor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , name_(config.get<std::string>("<xmlattr>.name"))
    , f0_Hz_(config.get<double>("<xmlattr>.f0_Hz"))
    , df_Hz_(config.get<double>("<xmlattr>.df_Hz"))
    , min_df_Hz_(config.get<double>("<xmlattr>.minDf_Hz"))
    , max_history_size_(config.get<size_t>("<xmlattr>.maxHistorySize")) {}
  
  ~tracking_goertzel_processor() {}

  void process_iq(service::sptr sp,
                  const_iterator i0,
                  const_iterator i1) {
    std::cout << "tracking_goertzel_processor::process_iq " << sp->stream_name()
              << " sample_rate_Hz=" <<   sp->sample_rate_Hz()
              << " center_frequency_Hz=" << sp->center_frequency_Hz()
              << " f0_Hz=" << f0_Hz_
              << std::endl;
    // set up a new filter
    const bool update_filter(filter_ ? (filter_->fs_Hz() != sp->sample_rate_Hz()) : true);
    if (update_filter)
      filter_ = tracking_goertzel_filter::make(sp->sample_rate_Hz(),
                                               f0_Hz_-sp->center_frequency_Hz(),
                                               df_Hz_,
                                               min_df_Hz_,
                                               max_history_size_);
    for (const_iterator i(i0); i!=i1; ++i) {
      filter_->update(*i);
      if (filter_->state_updated()) {
        const time_duration 
          dt(0,0,0, std::distance(i0, i)*time_duration::ticks_per_second()/sp->sample_rate_Hz());
        sp->put_result(result::make(name_,
                                    sp->approx_ptime()+dt,
                                    f0_Hz_,
                                    tracking_goertzel_filter::state2str(filter_->last_state()),
                                    double(filter_->period())/double(sp->sample_rate_Hz()),
                                    filter_->estimated_f_Hz(),
                                    filter_->estimated_df()));
      }
    }
  }

protected:
private:
  const std::string name_;
  const double      f0_Hz_;             // initial middle frequency
  const double      df_Hz_;             // initial df: [f0+-df]
  const double      min_df_Hz_;         // maximal frequency resolution
  const size_t      max_history_size_;  // maximal phase history size in seconds
  tracking_goertzel_filter::sptr filter_;
} ;

#endif // _TRACKING_GOERTZEL_PROCESSOR_HPP_cm130211_

