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
#ifndef _TRACKING_GOERTZEL_FILTER_HPP_cm20121104_
#define _TRACKING_GOERTZEL_FILTER_HPP_cm20121104_
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <deque>

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include "filter/goertzel.hpp"
#include "logging.hpp"

namespace detail {
  class value_and_error {
  public:
    value_and_error()
      : valid_(false)
      , value_(0.0)
      , rms_value_(1e10) {}

    value_and_error(double value,
		    double rms_value)
      : valid_(true)
      , value_(value)
      , rms_value_(rms_value) {}

    bool valid() const { return valid_; }
    double value() const { return value_; }
    double rms_value() const { return rms_value_; }

    friend std::ostream& operator<<(std::ostream& os, const value_and_error& ve) {
      if (not ve.valid()) return os << "(invalid)";
      return os << "( " << std::setprecision(15) << ve.value() << " +- " << ve.rms_value() << ")";
    }

  private:
    bool   valid_;
    double value_;
    double rms_value_;
  } ;

  class GoertzelFilterWithPhaseHistory : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<GoertzelFilterWithPhaseHistory> sptr;
    typedef std::complex<double> complex_t;
    typedef goertzel<complex_t> goertzel_type;
    typedef std::vector<double> phase_vector_type;

    virtual ~GoertzelFilterWithPhaseHistory() {}

    static sptr make() {
      return sptr(new GoertzelFilterWithPhaseHistory());
    }

  protected:
  private:
    GoertzelFilterWithPhaseHistory() {}

    goertzel_type     filter_;
    phase_vector_type phase_vector_;    
  } ;

} // namespace detail

class tracking_goertzel_filter : private boost::noncopyable {
public:
  typedef boost::shared_ptr<tracking_goertzel_filter> sptr;

  struct state {
    typedef enum {
      UNDEFINED  = 0,
      PEAK       = 1,
      MOVE_RIGHT = 2,
      MOVE_LEFT  = 3,
      REST       = 4
    } state_t;
  } ;

  static std::string state2str(state::state_t s) {
    static std::string stab[] = { "UNDEFINED", "PEAK", "MOVE_RIGHT", "MOVE_LEFT", "REST" };
    assert(s<5 && s>=0);
    return stab[s];
  }

  typedef std::complex<double> complex_t;

  static sptr make(double fs,              // sampling frequency (Hz)
		   double f0,              // center frequency (Hz)
		   double df,              // initial resolution (Hz): +- df
		   double min_df,          // maximal frequency resolution (Hz)
		   size_t max_hist_size) { // maximal size of history in seconds
    return sptr(new tracking_goertzel_filter(fs, f0, df, min_df, max_hist_size));
  }

  double         fs_Hz()         const { return fs_; }
  state::state_t last_state()    const { return last_state_; }
  bool           state_updated() const { return sample_counter_ == 0 && last_state() != state::UNDEFINED;  }
  size_t         period()        const { return period_; }
  const detail::value_and_error& estimated_f_Hz() const { return estimated_f_; }
  const detail::value_and_error& estimated_df()   const { return estimated_df_; }  

  void update(complex_t sample) {
    ++sample_counter_;
    if (sample_counter_ == period_) {
      sample_counter_ = 0;
      const double a(abs(left_.x()));
      const double b(abs(center_.x()));
      const double c(abs(right_.x()));

      if (b>a && b>c) { // peak
        const double sn(std::max(b/a, b/c));
        LOG_INFO(str(boost::format("Peak S/N=%f") % sn));
        phases_.push_back(std::make_pair(period_, std::arg(center_.x())));
        compute_exact_frequency();
        if (sn > 1.1 && period_ < max_period_ && history_size() > 5*period_) {
          period_ *= 2;
          left_  = goertzel_t(left_.kN()  + 1./period_);
          right_ = goertzel_t(right_.kN() - 1./period_);
        }
        last_state_ = state::PEAK;
      } else {
        phases_.clear();
	if (a>b && b>c) {
	  LOG_INFO("Move left");
	  if (period_ > min_period_ && last_state_ == state::MOVE_LEFT)
	    period_ /= 2;
	  left_   = goertzel_t(left_.kN()   - 1./period_);
	  center_ = goertzel_t(center_.kN() - 1./period_);
	  right_  = goertzel_t(right_.kN()  - 1./period_);
	  last_state_ = state::MOVE_LEFT;
	} else if (c>b && b>a) {
	  LOG_INFO("Move left");
	  if (period_ > min_period_ && last_state_ == state::MOVE_RIGHT)
	    period_ /= 2;
	  left_   = goertzel_t(left_.kN()   + 1./period_);
	  center_ = goertzel_t(center_.kN() + 1./period_);
	  right_  = goertzel_t(right_.kN()  + 1./period_);
	  last_state_ = state::MOVE_RIGHT;
	} else { // undefined
	  LOG_INFO("Rest");
	  if (period_ > min_period_ && last_state_ == state::REST)
	    period_ /= 2;
	  left_  = goertzel_t(center_.kN() - 1./period_);
	  right_ = goertzel_t(center_.kN() + 1./period_);
	  last_state_ = state::REST;
	}
      }
      left_.reset();
      center_.reset();
      right_.reset();
    }
    left_.update(sample);
    center_.update(sample);
    right_.update(sample);
  }

protected:
  size_t history_size() const {
    size_t hs(0);
    for (phase_vector_t::const_iterator i(phases_.begin()); i!= phases_.end(); ++i)
      hs += i->first;
    return hs;
  }

  void compute_exact_frequency() {
    // reset the state
    estimated_f_  = detail::value_and_error();
    estimated_df_ = detail::value_and_error();
    if (phases_.empty()) 
      return;
    
    while (history_size() > max_history_size_)
      phases_.pop_front();
    
    if (phases_.size() == 1) {
      estimated_f_ = detail::value_and_error(fs_*center_.kN(), 0.);
      return;
    }

    double_t  sum_1(0),  sum_x(0),  sum_xx(0);
    double_t dsum_1(0), dsum_x(0), dsum_xx(0);
    double_t last_df(0);

    const phase_vector_t uphases(unwrap(phases_));
    phase_vector_t::const_iterator end(uphases.end()-1);
    for (phase_vector_t::const_iterator i(uphases.begin()); i!=end; ++i) {
      phase_vector_t::const_iterator j(i+1);
      const double phase_diff(j->second - i->second);
      LOG_INFO(str(boost::format("compute_exact_frequency: phase_diff= %8.3f") % phase_diff));
      const double df(fs_*phase_diff / (2*M_PI*j->first));
      sum_1  += 1.;
      sum_x  += df;
      sum_xx += df*df;

      if (i != uphases.begin()) {
	const double ddf(2*(df-last_df)/(i->first + j->first));
	dsum_1  += 1.;
	dsum_x  += ddf;
	dsum_xx += ddf*ddf;
      }
      last_df= df;
    }
    assert(sum_1 != 0.0);
    const double df(sum_x/sum_1);
    const double rms_df(sum_1 > 1. ? sqrt(sum_xx/sum_1-df*df) : 0.0);
    estimated_f_ = detail::value_and_error(fs_*center_.kN()+df, rms_df);

    if (0.0 != dsum_1) {
      const double ddf(dsum_x/dsum_1);
      const double rms_ddf(dsum_1 > 1.0 ? sqrt(dsum_xx/dsum_1-ddf*ddf) : 0.0);
      estimated_df_ = detail::value_and_error(ddf, rms_ddf);
    }
  }

private:
  tracking_goertzel_filter(double fs,            // sampling frequency (Hz)
			   double f0,            // center frequency (Hz)
			   double df,            // initial resolution (Hz): +- df
			   double min_df,        // maximal frequency resolution (Hz)
			   size_t max_hist_size) // maximal size of history in seconds

    : fs_(fs)
    , period_(size_t(0.5+fs/df))
    , min_period_(period_)
    , max_period_(size_t(0.5+fs/min_df))
    , left_((f0-df)/fs)
    , center_(f0/fs)
    , right_((f0+df)/fs)
    , sample_counter_(0)
    , last_state_(state::UNDEFINED)
    , max_history_size_(size_t(0.5+fs*max_hist_size)) {}

  typedef goertzel<complex_t> goertzel_t;
  typedef std::deque<std::pair<size_t, double> > phase_vector_t;

  static phase_vector_t unwrap(const phase_vector_t& b) {
    phase_vector_t bu;
    if (b.size() == 0) return bu;
    bu.push_back(b[0]);
    if (b.size() == 1) return bu;
    double offset(0);
    for (size_t i=1;i<b.size(); ++i) {
      if (b[i].second-b[i-1].second >  M_PI) offset -= 2*M_PI;
      if (b[i].second-b[i-1].second < -M_PI) offset += 2*M_PI;
      bu.push_back(std::make_pair(b[i].first, b[i].second+offset));
    }
    return bu;
  }
  static double mod_2pi(double x) {
    while (x <= -M_PI) x += 2*M_PI;
    while (x >   M_PI) x -= 2*M_PI;
    return x;
  }

  double          fs_;
  size_t          period_;
  size_t          min_period_;
  size_t          max_period_;
  goertzel_t      left_;
  goertzel_t      center_;
  goertzel_t      right_;
  size_t          sample_counter_;
  state::state_t  last_state_;
  detail::value_and_error estimated_f_;
  detail::value_and_error estimated_df_;
  phase_vector_t  phases_;
  size_t          max_history_size_;
  size_t          history_size_;
} ;

#endif // _TRACKING_GOERTZEL_FILTER_HPP_cm20121104_
