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
#include <deque>
#include <iostream>
#include <iomanip>
#include <vector>

#include <boost/foreach.hpp>
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

  class gf_with_phase_hist : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<gf_with_phase_hist> sptr;
    typedef std::complex<double> complex_type;
    typedef goertzel<complex_type> goertzel_type;
    struct phase_data {
      phase_data(size_t dt=0,
                 double df=0,
                 double phase=0)
        : dt_(dt)
        , df_(df)
        , phase_(phase) {}

      size_t dt() const { return dt_; }
      double df() const { return df_; }
      double phase() const { return phase_; }

      size_t dt_;
      double df_;
      double phase_;
    } ;
    // (dt, phase)
    typedef std::deque<phase_data> phase_vector_t;

    virtual ~gf_with_phase_hist() {}

    static sptr make(std::string name, double kN, size_t period, size_t max_hist_size) {
      return sptr(new gf_with_phase_hist(name, kN, period, max_hist_size));
    }

    void set_df(double df) { df_ = df; }

    detail::value_and_error compute_f_est(const std::vector<double>& dfs, int idx_exclude = -1) const {
      // compute mean and rms
      double  sum_1(0),  sum_x(0),  sum_xx(0);
      int idx(0);
      BOOST_FOREACH(const double& df, dfs) {
        if (idx++ == idx_exclude) continue;
        sum_1  += 1;
        sum_x  += df;
        sum_xx += df*df;
      }
      const double df(sum_x/sum_1);
      const double rms_df(sum_1 > 1. ? sqrt(sum_xx/sum_1-df*df) : 0.0);
      return detail::value_and_error(filter_.kN()+df, rms_df);
    }

    void dump(size_t ccounter) const {
      std::cout << "updated[" << name_ << "]: " << last_phase_ << std::endl;
      if (phase_vector_.empty())
        return;
      
      std::vector<double> dfs(phase_vector_.size() -1);
      size_t counter(0);
      for (phase_vector_t::const_iterator i(phase_vector_.begin()), j(i+1);
           i != phase_vector_.end() && j != phase_vector_.end(); ++i, ++j, ++counter) {
        const double dt(0.5*(i->dt() + j->dt()));
        const double dphase(j->phase() - i->phase());
        dfs[counter] = j->df() + dphase/dt/(2*M_PI);
        if (i->df() != j->df() && counter>0)
          dfs[counter-1] += (j->df() - i->df());
//         if (i->df() != j->df() && counter>1)
//           dfs[counter-2] += (j->df() - i->df());

        std::cout << str(boost::format("\t%4d %9.3f %15.10f (%.10f %.10f) %15.10f %15.10f\n")
                         % j->dt()
                         % j->phase()
                         % filter_.kN()
                         % i->df()
                         % j->df()
                         % (j->df() + dphase/dt/(2*M_PI))
                         % (filter_.kN() +j->df() + dphase/dt/(2*M_PI))
                         );
      }
      std::vector<detail::value_and_error> rms(dfs.size());
      size_t   min_idx(-1);
      if (dfs.size() > 1) {
        double_t min_val(1e10);
        for (size_t i=0; i<dfs.size(); ++i) {
          rms[i] = compute_f_est(dfs, i);
          std::cout << "RRR: " << i << " " << rms[i] << std::endl;
          if (rms[i].rms_value() < min_val) {
            min_val = rms[i].rms_value();
            min_idx = i;
          }
        }
      }
      const value_and_error f_est(compute_f_est(dfs, min_idx));
      std::cout << "f_est = " << ccounter << " " << f_est << std::endl;
    }

    bool update(complex_type sample) {      
      filter_.update(sample);
      ++sample_counter_;

      bool phase_updated(false);
      if (sample_counter_ == period_) {
        while (history_size() > max_hist_size_)
          phase_vector_.pop_front();
    
        const double phase(std::arg(filter_.x()));
        last_amplitude_ = std::abs(filter_.x());
        // phase unwrapping
        if (!phase_vector_.empty()) {
          const double dphase(phase - last_phase_);
          if (dphase >  M_PI) offset_ -= 2*M_PI;
          if (dphase < -M_PI) offset_ += 2*M_PI;
        }
        phase_vector_.push_back(phase_data(period_, df_, offset_ + phase));
        last_phase_= phase;
        phase_updated= true;
        reset();
      }
      last_sample_ = sample;
      return phase_updated;
    }

    size_t size() const {
      return phase_vector_.size();
    }
    size_t history_size() const {
      size_t s(0);
      BOOST_FOREACH(const phase_vector_t::value_type& v, phase_vector_)
        s += v.dt();
      return s;
    }
    size_t period() const { return period_; }
    double amplitude() const { return last_amplitude_; }
    double kN() const { return filter_.kN(); }

    void reset() {
      filter_.reset();
      sample_counter_ = 0;
    }
    void update(double kN, size_t period) {
      period_ = period;
      if (filter_.kN() != kN) {
        std::cout << "UPDATE: " << filter_.kN() << " " << kN << filter_.kN() -kN << std::endl;
        filter_ = goertzel_type(kN);
        phase_vector_.clear();
      }
      reset();
    }

  protected:
  private:
    gf_with_phase_hist(std::string name, double kN, size_t period, size_t max_hist_size)
      : name_(name)
      , filter_(kN)
      , period_(period)
      , max_hist_size_(max_hist_size)
      , sample_counter_(0)
      , offset_(0)
      , last_amplitude_(0)
      , last_phase_(0)
      , last_sample_(0)
      , inhibit_reset_(false)
      , df_(0) {}
    
    std::string     name_;
    goertzel_type   filter_;
    phase_vector_t  phase_vector_;
    size_t          period_;         // in samples
    size_t          max_hist_size_;  // in samples
    size_t          sample_counter_;
    double          offset_;
    double          last_amplitude_;
    double          last_phase_;
    complex_type    last_sample_;
    bool            inhibit_reset_;
    double          df_; // shift in frequency
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
      REST       = 4,
      NUM_STATES
    } state_t;
  } ;
  static std::string state2str(state::state_t s) {
    static std::string stab[] = { "UNDEFINED", "PEAK", "MOVE_RIGHT", "MOVE_LEFT", "REST" };
    assert(s < state::NUM_STATES && s >= 0);
    return stab[s];
  }

  typedef enum {
    GF_LEFT   = 0,
    GF_CENTER = 1,
    GF_RIGHT  = 2,
    NUM_GF
  } gf_id_t;
  
  static std::string gf_id2str(gf_id_t id) {
    static std::string stab[] = { "GF_LEFT", "GF_CENTER", "GF_RIGHT" };
    assert(id < NUM_GF && id >= 0);
    return stab[id];
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
  bool           state_updated() const { return filters_updated_ && last_state() != state::UNDEFINED;  }
  size_t         period()        const { return period_; }
  const detail::value_and_error& estimated_f_Hz() const { return estimated_f_; }
  const detail::value_and_error& estimated_df()   const { return estimated_df_; }  

  void update(complex_t sample) {
    static size_t ccounter(0);
    ++ccounter;
    for (size_t i=0; i<NUM_GF; ++i)
      filters_updated_ = gfs_[i]->update(sample);

    if (!filters_updated_) return;

    double ampl[NUM_GF] = { 0 };
    std::cout << "AMPL: ";    
    for (size_t i=0; i<NUM_GF; ++i) {
      ampl[i] = gfs_[i]->amplitude();
      std::cout << "( " << ampl[i] << ", " << gfs_[i]->kN() << ", " << gfs_[i]->period() << " ) ";
    }
    std::cout << std::endl;
    gfs_[GF_CENTER]->dump(ccounter);

    // peak
    if (ampl[GF_CENTER] > ampl[GF_LEFT] &&
        ampl[GF_CENTER] > ampl[GF_RIGHT]) {
      const double sn(std::max(ampl[GF_CENTER]/ampl[GF_LEFT],
                               ampl[GF_CENTER]/ampl[GF_RIGHT]));
      LOG_INFO(str(boost::format("Peak S/N=%f") % sn));
      std::cout << str(boost::format("*** Peak S/N=%f") % sn) << std::endl;
      if (sn > 1.1 && period_ < max_period_ && gfs_[GF_CENTER]->history_size() > 5*period_) {
        period_ *= 2;
        const double kN_center(gfs_[GF_CENTER]->kN());
        gfs_[GF_LEFT]->update  (kN_center - 1./period_, period_);
//         gfs_[GF_CENTER]->update(kN_center- 1./period_,  period_);
        gfs_[GF_CENTER]->update(kN_center,              period_);
        gfs_[GF_RIGHT]->update (kN_center + 1./period_, period_);
      }
      last_state_ = state::PEAK;
    } else { 
      if (ampl[GF_LEFT]   > ampl[GF_CENTER] &&
          ampl[GF_CENTER] > ampl[GF_RIGHT]) { // 3 2 1      
        LOG_INFO("Move left");
        std::cout << "*** Move left" << std::endl;
        if (period_ > min_period_ && last_state_ == state::MOVE_LEFT) {
          period_ /= 2;
          const double kN_center(gfs_[GF_CENTER]->kN() - 1./period_);
          std::swap(gfs_[GF_CENTER], gfs_[GF_RIGHT]);
          gfs_[GF_LEFT]->update  (kN_center - 1./period_, period_);
          gfs_[GF_CENTER]->update(kN_center,              period_);
          gfs_[GF_RIGHT]->update (kN_center + 1./period_, period_);
        } else {
          std::swap(gfs_[GF_CENTER], gfs_[GF_RIGHT]);
          std::swap(gfs_[GF_LEFT],   gfs_[GF_CENTER]);
          gfs_[GF_LEFT] = detail::gf_with_phase_hist::make(gf_id2str(GF_LEFT), gfs_[GF_CENTER]->kN() - 1./period_, period_, max_history_size_);
        }
        last_state_ = state::MOVE_LEFT;
      } else if (ampl[GF_LEFT]   < ampl[GF_CENTER] &&
                 ampl[GF_CENTER] < ampl[GF_RIGHT]) { // 1 2 3
        LOG_INFO("Move right");
        std::cout << "*** Move right" << std::endl;
        if (period_ > min_period_ && last_state_ == state::MOVE_RIGHT) {
          std::cout << "\tA" << std::endl;
          period_ /= 2;
          const double kN_center(gfs_[GF_CENTER]->kN() + 1./period_);
          std::swap(gfs_[GF_LEFT], gfs_[GF_CENTER]);
          gfs_[GF_LEFT]->update  (kN_center - 1./period_, period_);
          gfs_[GF_CENTER]->update(kN_center,              period_);
          gfs_[GF_RIGHT]->update (kN_center + 1./period_, period_);
        } else {
          std::cout << "\tB" << std::endl;
          std::swap(gfs_[GF_LEFT],   gfs_[GF_CENTER]);
          std::swap(gfs_[GF_CENTER], gfs_[GF_RIGHT]);
          gfs_[GF_RIGHT] = detail::gf_with_phase_hist::make(gf_id2str(GF_RIGHT), gfs_[GF_CENTER]->kN() + 1./period_, period_, max_history_size_);
        }
        last_state_ = state::MOVE_RIGHT;
      } else {
        LOG_INFO("Rest");
        std::cout << "*** Rest" << std::endl;
        if (period_ > min_period_ && last_state_ == state::REST)
          period_ /= 2;
        const double kN_center(gfs_[GF_CENTER]->kN());
        gfs_[GF_LEFT]->update  (kN_center - 1./period_, period_);
        gfs_[GF_CENTER]->update(kN_center,              period_);
        gfs_[GF_RIGHT]->update (kN_center + 1./period_, period_);
        last_state_ = state::REST;
      }
    }    
    for (size_t i=0; i<NUM_GF; ++i)
      gfs_[i]->reset();

    gfs_[GF_LEFT]  ->set_df(gfs_[GF_CENTER]->kN() - gfs_[GF_LEFT]->kN());
    gfs_[GF_CENTER]->set_df(0);
    gfs_[GF_RIGHT] ->set_df(gfs_[GF_CENTER]->kN() - gfs_[GF_RIGHT]->kN());

//     compute_exact_frequency();
  }

protected:
//   size_t history_size() const {
//     size_t hs(0);
//     for (phase_vector_t::const_iterator i(phases_.begin()); i!= phases_.end(); ++i)
//       hs += i->dt();
//     return hs;
//   }
#if 0
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
#endif
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
    , filters_updated_(false)
    , last_state_(state::UNDEFINED)
    , max_history_size_(size_t(0.5+fs*max_hist_size)) {
    // max. history in samples
    gfs_[GF_LEFT]   = detail::gf_with_phase_hist::make(gf_id2str(GF_LEFT),  (f0-df)/fs, period_, max_history_size_);
    gfs_[GF_CENTER] = detail::gf_with_phase_hist::make(gf_id2str(GF_CENTER), f0    /fs, period_, max_history_size_);
    gfs_[GF_RIGHT]  = detail::gf_with_phase_hist::make(gf_id2str(GF_RIGHT), (f0+df)/fs, period_, max_history_size_);
  }

#if 0
  typedef goertzel<complex_t> goertzel_t;

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
#endif
  double          fs_;
  size_t          period_;
  size_t          min_period_;
  size_t          max_period_;
  detail::gf_with_phase_hist::sptr gfs_[NUM_GF];
  bool            filters_updated_;
  state::state_t  last_state_;
  detail::value_and_error estimated_f_;
  detail::value_and_error estimated_df_;
//   phase_vector_t  phases_;
  size_t          max_history_size_;
  size_t          history_size_;
} ;

#endif // _TRACKING_GOERTZEL_FILTER_HPP_cm20121104_
