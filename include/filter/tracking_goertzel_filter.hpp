// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2014 Christoph Mayer
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
    value_and_error& set_rms_value(double rms) {
      rms_value_ = rms;
      return *this;
    }
    value_and_error& operator*=(double f) {
      value_     *= f;
      rms_value_ *= f;
      return *this;
    }
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

      size_t dt_;    // time interval
      double df_;    // frequency offset
      double phase_; // phase [rad]
    } ;
    typedef std::deque<phase_data> phase_vector_t;

    virtual ~gf_with_phase_hist() {}

    static sptr make(std::string name, double kN, size_t period, size_t max_hist_size) {
      return sptr(new gf_with_phase_hist(name, kN, period, max_hist_size));
    }

    void set_df(double df) { df_ = df; }

    // idx_exclude < 0: no value is excluded
    detail::value_and_error compute_f_est(const std::vector<double>& dfs, int idx_exclude) const {
      // compute mean and rms
      double  sum_1(0), sum_x(0), sum_xx(0);
      for (size_t idx(0); idx<dfs.size(); ++idx) {
        if (int(idx) == idx_exclude) continue;
        const double df(dfs[idx]);
        sum_1  += 1;
        sum_x  += df;
        sum_xx += df*df;
      }
      const double df(sum_x/sum_1);
      const double rms_df(sum_1 > 1. ? sqrt(sum_xx/sum_1-df*df) : 0.0);
      return detail::value_and_error(filter_.kN()+df, rms_df);
    }

    // integrity method:
    //  - detects single outliers
    detail::value_and_error compute_f_est(const std::vector<double>& dfs) const {
      std::vector<detail::value_and_error> rms(dfs.size());
      int min_idx(-1);
      // outlier detection only with >=5 measurements:
      if (dfs.size() > 4) {
        rms[0] = compute_f_est(dfs, 0);
        double min_val(rms[0].rms_value());
        min_idx = 0;
        for (size_t i(1); i<dfs.size(); ++i) {
          rms[i] = compute_f_est(dfs, i);
          if (rms[i].rms_value() < min_val) {
            min_val = rms[i].rms_value();
            min_idx = i;
          }
        }

        // test if deviation is significant: ...
        double sum_1 (0);
        double sum_x (0);
        double sum_xx(0);
        for (size_t i(0); i<dfs.size(); ++i) {
          if (int(i) == min_idx) continue;
          const double x(rms[i].rms_value());
          sum_1  += 1;
          sum_x  += x;
          sum_xx += x*x;
        }
        const double mean_of_rms(sum_x/sum_1);
        const double rms_of_rms(sqrt(sum_xx/sum_1 - mean_of_rms*mean_of_rms));
        if (std::abs(min_val - mean_of_rms) < 3*rms_of_rms)
          min_idx = -1;
      }
      return compute_f_est(dfs, min_idx);
    }

    value_and_error compute_f_est() const {
      if (phase_vector_.empty()) {
        return value_and_error(0,0);
      }

      // make up vector with delta_fs
      std::vector<double> dfs(phase_vector_.size() -1);
      size_t counter(0);
      for (phase_vector_t::const_iterator i(phase_vector_.begin()), j(i+1);
           i != phase_vector_.end() && j != phase_vector_.end(); ++i, ++j, ++counter) {
        // time difference (Note: each measurement is made in the middle of its time interval!)
        const double dt(0.5*(i->dt() + j->dt()));
        // phase advance: each phase is ambiguous modulo df*2*M_PI TODO
        const double dphase(j->phase() - i->phase());
        dfs[counter] = j->df() + dphase/dt/(2*M_PI);
        if (i->df() != j->df() && counter>0)
          dfs[counter-1] += (j->df() - i->df());

#if 0
        std::cout << str(boost::format("\t%4d %9.3f %8.4e %15.10f (%.10f %.10f) %15.10f %15.10f\n")
                         % j->dt()
                         % j->phase()
                         % dfs[counter]
                         % filter_.kN()
                         % i->df()
                         % j->df()
                         % (j->df() + dphase/dt/(2*M_PI))
                         % (filter_.kN() +j->df() + dphase/dt/(2*M_PI))
                         );
#endif
      }
      if (dfs.empty())
        return value_and_error(0,0);

      // apply integrity method
      return compute_f_est(dfs);
    }

    void dump(size_t ccounter) const {
      if (phase_vector_.empty())
        return;

//       const value_and_error f_est(compute_f_est());
      // std::cout << "f_est = " << ccounter << " " << f_est << std::endl;
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

    // length of phase history
    size_t size() const {
      return phase_vector_.size();
    }
    // size in samples
    size_t history_size() const {
      size_t s(0);
      for (auto const& v : phase_vector_)
        s += v.dt();
      return s;
    }
    size_t period() const { return period_; }
    double amplitude() const { return last_amplitude_; }
    double kN() const { return filter_.kN(); }

    void reset(std::string name="") {
      if (name != "")
        name_ = name;
      filter_.reset();
      sample_counter_ = 0;
    }
    void update(double kN, size_t period) {
      period_ = period;
      if (filter_.kN() != kN) {
        filter_ = goertzel_type(kN);
        phase_vector_.clear();
      }
      reset();
    }

  protected:
    bool inhibit_reset() const { return inhibit_reset_; }

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
    const size_t    max_hist_size_;  // in samples
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

  static size_t state_delay(state::state_t s) {
    static size_t sdtab[] = { 0, 0, 2, 2, 1 }; // by these amounts state changes are delayed
    return sdtab[s];
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
		   size_t max_hist_size,   // maximal size of history in seconds
                   size_t max_num_without_lock) {
    return sptr(new tracking_goertzel_filter(fs, f0, df, min_df, max_hist_size, max_num_without_lock));
  }

  double         fs_Hz()         const { return fs_; }
  state::state_t last_state()    const { return last_state_; }
  bool           state_updated() const { return filters_updated_ && last_state() != state::UNDEFINED;  }
  size_t         period()        const { return period_; }
  const detail::value_and_error& estimated_f_Hz() const { return estimated_f_; }
  const detail::value_and_error& estimated_df()   const { return estimated_df_; }
  double        delta_time_sec() const { return delta_time_sec_; }

  bool update(complex_t sample) {
    static size_t ccounter(0);
    ++ccounter;

    // (1) run all Goertzel filters
    for (size_t i(0); i<NUM_GF; ++i)
      filters_updated_ = gfs_[i]->update(sample);

    // (2) if they did not finish return
    if (!filters_updated_) return false;

    // (3) change the filter according to the measured amplitudes

    double ampl[NUM_GF] = { 0 };
    // std::cout << "AMPL: ";
    for (size_t i(0); i<NUM_GF; ++i) {
      ampl[i] = gfs_[i]->amplitude();
      // std::cout << "( " << ampl[i] << ", " << gfs_[i]->kN() << ", " << gfs_[i]->period() << " ) ";
    }
    // std::cout << std::endl;
    gfs_[GF_CENTER]->dump(ccounter);

    // compute the frequency
    estimated_f_ = gfs_[GF_CENTER]->compute_f_est();
    delta_time_sec_ = -0.5*double(gfs_[GF_CENTER]->history_size())/fs_;
    if (0.0 == estimated_f_.value())
      estimated_f_ = detail::value_and_error(gfs_[GF_CENTER]->kN(), 0);
    if (0.0 == estimated_f_.rms_value())
      estimated_f_.set_rms_value(gfs_[GF_CENTER]->kN() - gfs_[GF_LEFT]->kN());

    // conversion: normalized frequency -> absolute frequency
    estimated_f_ *= fs_;
    // std::cout << "F_est= " << ccounter-gfs_[GF_CENTER]->history_size()/2 << " " << estimated_f_ << std::endl;

    // peak
    if (ampl[GF_CENTER] > ampl[GF_LEFT] &&
        ampl[GF_CENTER] > ampl[GF_RIGHT]) {
      const double sn(std::max(ampl[GF_CENTER]/ampl[GF_LEFT],
                               ampl[GF_CENTER]/ampl[GF_RIGHT]));
      LOG_INFO(str(boost::format("Peak S/N=%f") % sn));
      // std::cout << str(boost::format("*** Peak S/N=%f") % sn) << std::endl;
      if (sn > 1.1 && period_ < max_period_ && gfs_[GF_CENTER]->history_size() > 5*period_) {
        period_ *= 2;
        const double kN_center(gfs_[GF_CENTER]->kN());
        gfs_[GF_LEFT]->update  (kN_center - 1./period_, period_);
        gfs_[GF_CENTER]->update(kN_center,              period_);
        gfs_[GF_RIGHT]->update (kN_center + 1./period_, period_);
      }
      last_state_ = state::PEAK;
    } else {
      if (ampl[GF_LEFT]   > ampl[GF_CENTER] &&
          ampl[GF_CENTER] > ampl[GF_RIGHT]  &&
          ampl[GF_LEFT] / ampl[GF_CENTER] > 1.1) { // 3 2 1
        LOG_INFO("Move left");
        // std::cout << "*** Move left" << std::endl;
        if (period_ > min_period_ && last_state() == state::MOVE_LEFT) {
          period_ /= 2;
          const double kN_center(gfs_[GF_CENTER]->kN() - 1./period_);
          if (kN_center - 1./period_ > k_min_ &&
              kN_center + 1./period_ < k_max_) {
            std::swap(gfs_[GF_CENTER], gfs_[GF_RIGHT]);
            gfs_[GF_LEFT]->update  (kN_center - 1./period_, period_);
            gfs_[GF_CENTER]->update(kN_center,              period_);
            gfs_[GF_RIGHT]->update (kN_center + 1./period_, period_);
          }
        } else {
          const double kN_center(gfs_[GF_LEFT]->kN());
          if (kN_center - 1./period_ > k_min_ &&
              kN_center + 1./period_ < k_max_) {
            std::swap(gfs_[GF_CENTER], gfs_[GF_RIGHT]);
            std::swap(gfs_[GF_LEFT],   gfs_[GF_CENTER]);
            gfs_[GF_LEFT] =
              detail::gf_with_phase_hist::make(gf_id2str(GF_LEFT), kN_center - 1./period_, period_, max_history_size_);
          }
        }
        last_state_ = state::MOVE_LEFT;
      } else if (ampl[GF_LEFT]   < ampl[GF_CENTER] &&
                 ampl[GF_CENTER] < ampl[GF_RIGHT]  &&
                 ampl[GF_RIGHT]/ampl[GF_CENTER] > 1.1) { // 1 2 3
        LOG_INFO("Move right");
        // std::cout << "*** Move right" << std::endl;
        if (period_ > min_period_ && last_state() == state::MOVE_RIGHT) {
          // std::cout << "\tA" << std::endl;
          period_ /= 2;
          const double kN_center(gfs_[GF_CENTER]->kN() + 1./period_);
          if (kN_center - 1./period_ > k_min_ &&
              kN_center + 1./period_ < k_max_) {
            std::swap(gfs_[GF_LEFT], gfs_[GF_CENTER]);
            gfs_[GF_LEFT]->update  (kN_center - 1./period_, period_);
            gfs_[GF_CENTER]->update(kN_center,              period_);
            gfs_[GF_RIGHT]->update (kN_center + 1./period_, period_);
          }
        } else {
          // std::cout << "\tB" << std::endl;
          const double kN_center(gfs_[GF_RIGHT]->kN());
          if (kN_center - 1./period_ > k_min_ &&
              kN_center + 1./period_ < k_max_) {
            std::swap(gfs_[GF_LEFT],   gfs_[GF_CENTER]);
            std::swap(gfs_[GF_CENTER], gfs_[GF_RIGHT]);
            gfs_[GF_RIGHT] =
              detail::gf_with_phase_hist::make(gf_id2str(GF_RIGHT), kN_center + 1./period_, period_, max_history_size_);
          }
        }
        last_state_ = state::MOVE_RIGHT;
      } else {
        LOG_INFO(str(boost::format("Rest %d %d") % period_ % min_period_));
        if (period_ > min_period_)// && last_state() == state::REST)
          period_ /= 2;
        const double kN_center(gfs_[GF_CENTER]->kN());
        if (kN_center - 1./period_ > k_min_ &&
            kN_center + 1./period_ < k_max_) {
          gfs_[GF_LEFT]->update  (kN_center - 1./period_, period_);
          gfs_[GF_CENTER]->update(kN_center,              period_);
          gfs_[GF_RIGHT]->update (kN_center + 1./period_, period_);
        }
        last_state_ = state::REST;
      }
    }


    // reset all filters
    for (size_t i(0); i<NUM_GF; ++i)
      gfs_[i]->reset(gf_id2str(gf_id_t(i)));

    gfs_[GF_LEFT]  ->set_df(gfs_[GF_CENTER]->kN() - gfs_[GF_LEFT]->kN());
    gfs_[GF_CENTER]->set_df(0);
    gfs_[GF_RIGHT] ->set_df(gfs_[GF_CENTER]->kN() - gfs_[GF_RIGHT]->kN());

    // count how many steps there were not in the PEAK state
    if (last_state_ != state::PEAK)
      ++num_without_lock_;
    else
      num_without_lock_ = 0;

    if (num_without_lock_ > max_num_without_lock_)
      reset();

    return true;
  }

protected:
  size_t state_counter() const { return state_counter_; }
  size_t history_size() const { return history_size_; }
private:
  tracking_goertzel_filter(double fs,            // sampling frequency (Hz)
			   double f0,            // center frequency (Hz)
			   double df,            // initial resolution (Hz): +- df
			   double min_df,        // maximal frequency resolution (Hz)
			   size_t max_hist_size, // maximal size of history in seconds
                           size_t max_num_without_lock) // max. number of iterations without lock

    : fs_(fs)
    , f0_(f0)
    , df_(df)
    , min_period_(size_t(0.5+fs/df))
    , max_period_(size_t(0.5+fs/min_df))
    , k_min_((f0-df)/fs)
    , k_max_((f0+df)/fs)
    , max_history_size_(size_t(0.5+fs*max_hist_size))
    , max_num_without_lock_(max_num_without_lock) {
    reset();
  }

  void reset() {
    period_           = size_t(0.5+fs_/df_);
    last_state_       = state::UNDEFINED;
    filters_updated_  = false;
    estimated_f_      = detail::value_and_error(f0_, df_);
    delta_time_sec_   = 0;
    num_without_lock_ = 0;
    const double f[NUM_GF] = { f0_-df_, f0_ , f0_+df_ };
    for (size_t i(0); i<NUM_GF; ++i)
      gfs_[i]   = detail::gf_with_phase_hist::make(gf_id2str(gf_id_t(i)),  f[i]/fs_, period_, max_history_size_);
  }

  const double    fs_;
  const double    f0_;
  const double    df_;
  size_t          period_;
  size_t          min_period_;
  size_t          max_period_;
  const double    k_min_; // lower bound in normalized frequency
  const double    k_max_; // upper bound in normalized frequency
  detail::gf_with_phase_hist::sptr gfs_[NUM_GF];
  bool            filters_updated_;
  state::state_t  last_state_;
  size_t          state_counter_;  // used to delay switching the state
  detail::value_and_error estimated_f_;
  detail::value_and_error estimated_df_;
  double          delta_time_sec_; // time offset of frequency measurement [sec]
  //   phase_vector_t  phases_;
  const size_t    max_history_size_;
  size_t          history_size_;
  size_t          num_without_lock_;
  const size_t    max_num_without_lock_;
} ;

#endif // _TRACKING_GOERTZEL_FILTER_HPP_cm20121104_
