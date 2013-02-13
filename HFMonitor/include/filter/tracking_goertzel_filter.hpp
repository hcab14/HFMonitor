// -*- c++ -*-
// $Id$

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
  bool           state_updated() const {
    return sample_counter_ == 0 && last_state() != state::UNDEFINED;
  }
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
      std::cout << fs_/period_ << " "
		<< fs_*left_.kN() << " [" << abs(left_.x()) << " " << arg(left_.x()) << "], ";
      std::cout << fs_*center_.kN() << " [" << abs(center_.x()) << " " << arg(center_.x()) << "], ";
      std::cout << fs_*right_.kN() << " [" << abs(right_.x()) << " " << arg(right_.x()) << "], ";

      if (b>a && b>c) { // peak
	std::cout << "peak " << std::max(b/a, b/c);
	phases_.push_back(std::make_pair(period_, std::arg(center_.x())));
	compute_exact_frequency();
	if (std::max(b/a, b/c) > 1.1 && period_ < max_period_ && history_size() > 5*period_) {
	  period_ *= 2;
	  left_  = goertzel_t(left_.kN()  + 1./period_);
	  right_ = goertzel_t(right_.kN() - 1./period_);
	}
	last_state_ = state::PEAK;
      } else {
	phases_.clear();
	if (a>b && b>c) {
	  std::cout << "move left";
	  if (period_ > min_period_ && last_state_ == state::MOVE_LEFT)
	    period_ /= 2;
	  left_   = goertzel_t(left_.kN()   - 1./period_);
	  center_ = goertzel_t(center_.kN() - 1./period_);
	  right_  = goertzel_t(right_.kN()  - 1./period_);
	  last_state_ = state::MOVE_LEFT;
	} else if (c>b && b>a) {
	  std::cout << "move right";
	  if (period_ > min_period_ && last_state_ == state::MOVE_RIGHT)
	    period_ /= 2;
	  left_   = goertzel_t(left_.kN()   + 1./period_);
	  center_ = goertzel_t(center_.kN() + 1./period_);
	  right_  = goertzel_t(right_.kN()  + 1./period_);
	  last_state_ = state::MOVE_RIGHT;
	} else { // undefined
	  std::cout << "rest";
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
      std::cout << std::endl;
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
      estimated_f_ = detail::value_and_error(fs_*center_.kN(), 1e10);
      return;
    }

    double_t  sum_1(0),  sum_x(0),  sum_xx(0);
    double_t dsum_1(0), dsum_x(0), dsum_xx(0);
    double_t last_df(0);
    phase_vector_t::const_iterator end(phases_.end()-1);
    for (phase_vector_t::const_iterator i(phases_.begin()); i!=end; ++i) {
      phase_vector_t::const_iterator j(i+1);
      const double phase_diff(mod_2pi(j->second - i->second));
      std::cout << "a---- " <<  phase_diff << " " << j->second  - i->second << std::endl;
      const double df(fs_*phase_diff / (2*M_PI*j->first));
      sum_1  += 1.;
      sum_x  += df;
      sum_xx += df*df;

      if (i != phases_.begin()) {
	const double ddf(2*(df-last_df)/(i->first + j->first));
	dsum_1  += 1;
	dsum_x  += ddf;
	dsum_xx += ddf*ddf;
      }
      last_df= df;
    }
    const double df(sum_x/sum_1);
    const double rms_df(sqrt(sum_xx/sum_1-df*df));
    estimated_f_ = detail::value_and_error(fs_*center_.kN()+df, rms_df);

    if (0.0 != dsum_1) {
      const double ddf(dsum_x/dsum_1);
      const double rms_ddf(sqrt(dsum_xx/dsum_1-ddf*ddf));
      estimated_df_ = detail::value_and_error(ddf, rms_ddf);
    }
    std::cout << "\n## f,df: " << estimated_f_ << " " << estimated_df_ << std::endl;
  }

private:
  static double mod_2pi(double x) {
    while (x <= -M_PI) x += 2*M_PI;
    while (x >   M_PI) x -= 2*M_PI;
    return x;
  }
  tracking_goertzel_filter(double fs,            // sampling frequency (Hz)
			   double f0,            // center frequency (Hz)
			   double df,            // initial resolution (Hz): +- df
			   double min_df,        // maximal frequency resolution (Hz)
			   size_t max_hist_size) // maximal size of history in seconds

    : fs_(fs)
    , period_(size_t(0.5+fs/df+0.5))
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
