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
#include <cmath>
#include <iostream>
#include <fstream>
#include <iterator>
#include <numeric>
#include <boost/format.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "logging.hpp"
#include "FFT.hpp"
#include "filter/goertzel.hpp"

std::ostream& operator<<(std::ostream& os, const std::vector<float>& v) {
  os << "[";
  std::copy(v.begin(), v.end(), std::ostream_iterator<float>(os, " "));
  return os << "]";
}
std::ostream& operator<<(std::ostream& os, const std::map<double, double>& v) {
  os << "[ ";
  for (std::map<double,double>::const_iterator i(v.begin()), iend(v.end()); i!=iend; ++i)
    os << "(" << i->first << "," << i->second << ") ";
  return os << "]";
}


class demod_alpha_processor :  processor::base_iq  {
public:
  typedef boost::shared_ptr<demod_alpha_processor> sptr;
  typedef boost::posix_time::time_duration time_duration;

  typedef goertzel<std::complex<float> > goertzel_type;

  class result_alpha : public processor::result_base {
  public:
    typedef boost::shared_ptr<result_alpha> sptr;
    
    virtual ~result_alpha() {}
    static sptr make(std::string name,
                     ptime t,
                     float bkgd,
                     float sig[6],
                     float phases[6],
                     int   offset) {
      return sptr(new result_alpha(name, t, bkgd, sig, phases, offset));
    }


    virtual std::ostream& dump_header(std::ostream& os) const {      
      return os
        << "# Time_UTC bkgd[dB] Amplitudes[dB] Phases[rad] ";
    }
    virtual std::ostream& dump_data(std::ostream& os) const {
      const float threshold_db = 1.4;
      os << boost::format("%6.2f ") % bkgd_;
      for (int i=0; i<6; ++i)
        os << boost::format("%5.2f ") % (sig_[i] > threshold_db ? sig_[i]    : 0.0);
      for (int i=0; i<6; ++i)
        os << boost::format("%6.3f ") % (sig_[i] > threshold_db ? phases_[i] : 0.0);
      os << (locked_ ? " STN_LOCKED" : " STN_UNLOCKED");
      return os;
    }
    
  protected:
    static float s2db(float s) {
      return 10*std::log10(s);
    }
  private:
    result_alpha(std::string name,
                 ptime  t,
                 float bkgd,
                 float sig[6],
                 float phases[6],
                 int   offset)
      : result_base(name, t)
      , bkgd_(s2db(bkgd))
      , locked_(offset!=-1) {
      for (int i=0; i<6; ++i) {
        const int j = (locked_ ? ((i+offset)%6) : i);
        sig_[i]  = s2db(sig[j]);
        phases_[i] = phases[j];
      }
    }
    
    float bkgd_;
    float sig_[6];
    float phases_[6];
    bool  locked_;
  } ; 


  enum {
    UNINITIALIZED = -1,
    INITIALIZED,
    LOCKED
  };

  demod_alpha_processor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , state_(UNINITIALIZED)
    , sample_counter_(0)
    , period_(1)
    , dt_(0.05)
    , buf_counter_(0)
    , buf_size_(1)
    , periodPhase_(1)
    , periodSlot_(1)
    , counterPhaseMeasurement_(0)
    , counterSlot_(0)
    , counterCarrier_(0)
    , counterPhaseShifts_(0)
    , alpha_(0.5)
    , s_last_(1) {}

  void process_iq(processor::service_iq::sptr sp, const_iterator i0, const_iterator i1) {
#if 0
    std::cout << "process_iq nS=" << std::distance(i0, i1) 
              << " " << sp->id()
              << " " << sp->approx_ptime()
              << " " << sp->sample_rate_Hz()
              << " " << sp->center_frequency_Hz()
              << " " << sp->offset_ppb()
              << std::endl;
#endif
//     const ssize_t length(std::distance(i0, i1));

    const std::string names[3] = { "F1", "F2", "F3" };

    const double fb(1e6/double(7*3*64)); // ALPHA base frequency
    const double f[3] = {
      16*fb, 17*fb, 20*fb
    };
    const double df[3] = {
      std::fmod(f[0], 1.0/3.6),
      std::fmod(f[1], 1.0/3.6),
      std::fmod(f[2], 1.0/3.6)
    };

    if (state_ == UNINITIALIZED) {

      buf_counter_ = 0;
      buf_size_    = int(0.5+3.6/dt_);
      
      for (int i=0; i<3; ++i) {
        // std::cout << "f[" << i << "] " << f[i] << std::endl;
        gf_[i].set_parameter(-(f[i]-sp->center_frequency_Hz())/sp->sample_rate_Hz());
        gfPhase_[i].set_parameter(-(f[i]-sp->center_frequency_Hz())/sp->sample_rate_Hz());
        buf_[i].resize(buf_size_);
        for (size_t j=0; j<buf_size_; ++j)
          buf_[i][j] = 0;
      }

      period_      = size_t(0.5 + sp->sample_rate_Hz() * dt_);
      periodPhase_ = size_t(0.5 + sp->sample_rate_Hz() * 0.4);
      periodSlot_  = size_t(0.5 + sp->sample_rate_Hz() * 0.6);

      counterSlot_ = counterPhaseMeasurement_ = counterCarrier_ = counterPhaseShifts_ = 0;

      alpha_ = 1./(1.+sp->sample_rate_Hz() * 0.4);
      s_last_ = 0.01;

      state_ = INITIALIZED;
    }
    
    for (const_iterator i=i0; i!=i1; ++i) {      
      ++sample_counter_;
      const bool b = abs(*i) < 8*s_last_;
      s_last_ = (1.-alpha_)*s_last_ + b*alpha_*std::abs(*i) + (1-b)*alpha_*s_last_;
      const std::complex<float> s(b ? *i  : std::complex<float>(0));
      for (int j=0; j<3; ++j)
        gf_[j].update(s * FFT::WindowFunction::Hamming<float>(period_)(sample_counter_));

      if (sample_counter_ == period_) {
        sample_counter_ = 0;
        const double alpha=0.1;
        for (int j=0; j<3; ++j) {
          buf_[j][buf_counter_] = (1-alpha)*buf_[j][buf_counter_] + alpha*std::abs(gf_[j].x());
          gf_[j].reset();
        }

        ++buf_counter_;
        if (buf_counter_ == buf_size_) {
          buf_counter_ = 0;
          int offset[3];
          for (int j=0; j<3; ++j) {
            offset[j] = check_lock(buf_[j], sig_[j], bkgd_[j]);
#if 0
            std::cout << "buf[" << j << "]  " << offset[j] << " (" <<
              sig_[j][0] << "," << 
              sig_[j][1] << "," << 
              sig_[j][2] << "," << 
              sig_[j][3] << "," << 
              sig_[j][4] << "," << 
              sig_[j][5] << ")"
                      << " state="<< state_ << std::endl;
#endif
          }
          if (state_ != LOCKED &&
              offset[0] == offset[1] &&
              offset[0] == offset[2]) {
            state_ = LOCKED;
            counterPhaseMeasurement_ = -offset[0] * int(sp->sample_rate_Hz()) * dt_;
            // std::cout << "counterPhaseMeasurement=" << counterPhaseMeasurement_ << " " << periodPhase_ << std::endl;
          }
        }
      }    

      if (state_ == LOCKED) {
        if (counterPhaseMeasurement_ >= 0) {
          for (int j=0; j<3; ++j)
            gfPhase_[j].update(s * FFT::WindowFunction::Hamming<float>(periodPhase_)(counterPhaseMeasurement_));
        }
        ++counterPhaseMeasurement_;
        
        if (counterPhaseMeasurement_ == periodPhase_) {
          for (int j=0; j<3; ++j) {
            phases_[j][counterSlot_] = pmPi(std::arg(gfPhase_[j].x()) -
                                            counterCarrier_*2*M_PI*df[j]*3.6 +
                                            counterPhaseShifts_/24000.0*17*2*M_PI);
            // std::cout << "j=" << j
            //           << " slot="  << counterSlot_
            //           << " phase= " << phases_[j][counterSlot_] << " " << std::abs(gfPhase_[j].x())
            //           << std::endl;
          }
          ++counterSlot_;

          if (counterSlot_ == 6) {
            counterSlot_ = 0;

            // synchronize
            const int off = find_stations();
            // std::cout << "find_stations off = " << off << std::endl;
            for (int j=0; j<3; ++j) {
              const time_duration
                dt(0,0,0, std::distance(i0, i)*time_duration::ticks_per_second()/sp->sample_rate_Hz());
              const time_duration dt2(0,0,0, (-3.6+off*0.6+0.2)*time_duration::ticks_per_second());
              result_alpha::sptr rp = result_alpha::make("ALPHA_"+names[j],sp->approx_ptime()+dt+dt2, bkgd_[j], sig_[j], phases_[j], off);
              std::cout << rp->approx_ptime() << " " << rp->name() << " ";
              rp->dump_data(std::cout);
              sp->put_result(rp);
              std::cout << std::endl;
            }
            // TBD
          }
        }
        if (counterPhaseMeasurement_ == periodSlot_) {
          counterCarrier_ += 1;
          counterCarrier_ %= 7;

          counterPhaseShifts_ += 1;
          counterPhaseShifts_ %= 24000;

          counterPhaseMeasurement_ = 0;
          for (int j=0; j<3; ++j)
            gfPhase_[j].reset();
        }
      }
    }    
  }

  int find_stations() const {
    const float threshold_db = 1.4;
    bool b[3][6];
    for (int i=0; i<3; ++i)
      for (int j=0; j<6; ++j)
        b[i][j] = 10*std::log10(sig_[i][j]) > threshold_db;

    // for (int i=0; i<3; ++i)
    //   std::cout << "find_stations "
    //             << b[i][0] << b[i][1] << b[i][2] << b[i][3] << b[i][4] << b[i][5] << std::endl;

    const bool no[3][6] = { {1,0,0,0,0,0},
                            {0,1,0,0,0,0},
                            {0,0,1,0,0,0} };
    
    const bool kr[3][6] = { {0,0,1,0,0,0},
                            {0,0,0,1,0,0},
                            {1,0,0,0,0,0} };
    
    const bool kh[3][6] = { {0,0,0,1,0,0},
                            {0,0,1,0,0,0},
                            {0,1,0,0,0,0} };
    
    int off[6] = { 0,0,0, 0,0,0 };
    find_stations_offset(b, no, off);
    find_stations_offset(b, kr, off);
    find_stations_offset(b, kh, off);

    int *max = std::max_element(off, off+6);
    return (*max == 0
            ? -1
            : std::distance(off, max));
  }

  void find_stations_offset(const bool b[3][6], const bool s[3][6], int off[6]) const {
    int f[6] = { 0,0,0, 0,0,0 };
    for (int offset=0; offset<6; ++offset) {
      for (int i=0; i<3; ++i)
        for (int j=0; j<6; ++j)
          f[offset] += (b[i][(j+offset)%6] & s[i][j]);
    }
#if 0
    std::cout << "find_stations_offset "
              << f[0] << " " << f[1] << " " << f[2] << " " << f[3] << " " << f[4] << " " << f[5] << " "  << std::endl;
#endif
    for (int i=0; i<6; ++i)
      off[i] += (f[i] == 3);
  }

  // x -> [-pi..pi)
  static double pmPi(double x) {
    while (x <  -M_PI)
      x += 2*M_PI;
    while (x >= +M_PI)
      x -= 2*M_PI;
    return x;
  }

  int check_lock(const std::vector<float>& buf, float sig[6], float &bkgd) const {
    const size_t len_0 = size_t(0.5+0.4/dt_);
    const size_t len_2 = size_t(0.5+0.6/dt_);

    // std::cout << "check_lock: " << buf << std::endl;
    
    const int n = buf.size();
    std::vector<float> corr(buf.size(), 0);
    for (size_t i=0; i<buf.size(); ++i)
      for (size_t j=0; j<buf.size(); ++j)
        corr[i] += buf[(i+j)%n] * ((j%len_2) < len_0);
    
    std::vector<float>::iterator im = std::max_element(corr.begin(), corr.end());
    const int offset = (std::distance(corr.begin(), im) % len_2);

    int n_bkgd = 0;
    int n_sig[6] = { 0,0,0, 0,0,0 };

    bkgd = 0;
    for (int i=0; i<6; ++i)
      sig[i] = 0;

    for (int i=0; i<n; ++i) {
      const int j = ((offset+i)%n);
      const bool is_bkgd((i%len_2) >= len_0);
      const bool is_sig ((i%len_2) <  len_0);
      n_bkgd         += is_bkgd;
      bkgd           += is_bkgd * buf[j];
      n_sig[i/len_2] += is_sig;
      sig[i/len_2]   += is_sig  * buf[j];
      // std::cout << "CCC " << i << " " << j << " " << is_sig << " " << is_bkgd << " " << buf[j] << std::endl;
    }

    bkgd /= n_bkgd;
    for (int i=0; i<6; ++i)
      sig[i] /= (n_sig[i] * bkgd);

    return offset;
  }

  struct s2db {
    double operator()(double c) const {
      return 10*std::log10(c);
    }
  } ;

private:
  int           state_;
  size_t        sample_counter_;
  size_t        period_;                  // dt_ samples 
  double        dt_;                      // 0.05 sec used for pulse synchronization
  goertzel_type gf_[3];                   // Goertzel filters for pulse synchronization
  size_t        buf_counter_;
  size_t        buf_size_;                // 3.6 sec / dt_
  std::vector<float> buf_[3];             // average field strength in last 3.6 sec interval

  int           periodPhase_;             // 0.4 sec
  int           periodSlot_;              // 0.6 sec
  int           counterPhaseMeasurement_; // [0..periodSlot-1]
  int           counterSlot_;             // [0..5]
  goertzel_type gfPhase_[3];              // Goertzel filters for phase measurement (0.4 sec)
  int           counterCarrier_;          // [0-7] phase advance in 3.6 sec
  int           counterPhaseShifts_;      // [0-23999] phase progression of 102*2pi/24h = 17*24/4h

  float         bkgd_[3];         // background
  float         sig_[3][6];       // signal in F123 in 6 slots each
  float         phases_[3][6];    // phases in F123 in 6 slots each

  float         alpha_;     // low-pass filter parameter (static crashes)
  float         s_last_;    // low-pass filter state
} ;
