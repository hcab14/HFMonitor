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
                     float phases[6]) {
      return sptr(new result_alpha(name, t, bkgd, sig, phases));
    }


    virtual std::ostream& dump_header(std::ostream& os) const {      
      return os
        << "# Time_UTC bkgd[dB] Amplitudes[dB] Phases[rad] ";
    }
    virtual std::ostream& dump_data(std::ostream& os) const {
      os << boost::format("%7.2f ") % s2db(bkgd_);
      for (int i=0; i<6; ++i)
        os << boost::format("%7.2f ") % s2db(sig_[i]);
      for (int i=0; i<6; ++i)
        os << boost::format("%6.3f ") % (s2db(sig_[i]) > 1.4 ? phases_[i] : 0.0);
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
                 float phases[6])
      : result_base(name, t)
      , bkgd_(bkgd) {
      std::copy(sig,    sig   +6, sig_);
      std::copy(phases, phases+6, phases_);
    }
    
    float bkgd_;
    float sig_[6];
    float phases_[6];
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
    , counterPhase_(0)
    , counterSlot_(0)
    , counterCarrier_(0) {}

  void process_iq(processor::service_iq::sptr sp, const_iterator i0, const_iterator i1) {

    std::cout << "process_iq nS=" << std::distance(i0, i1) 
              << " " << sp->id()
              << " " << sp->approx_ptime()
              << " " << sp->sample_rate_Hz()
              << " " << sp->center_frequency_Hz()
              << " " << sp->offset_ppb()
              << std::endl;

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
        std::cout << "f[" << i << "] " << f[i] << std::endl;
        gf_[i].set_parameter(-(f[i]-sp->center_frequency_Hz())/sp->sample_rate_Hz());
        gfPhase_[i].set_parameter(-(f[i]-sp->center_frequency_Hz())/sp->sample_rate_Hz());
        buf_[i].resize(buf_size_);
        for (int j=0; j<buf_size_; ++j)
          buf_[i][j] = 0;
      }

      period_      = size_t(0.5 + sp->sample_rate_Hz() * dt_);
      periodPhase_ = size_t(0.5 + sp->sample_rate_Hz() * 0.4);
      periodSlot_  = size_t(0.5 + sp->sample_rate_Hz() * 0.6);

      counterSlot_ = counterPhase_ = counterCarrier_ = 0;

      state_ = INITIALIZED;
    }
    
    for (const_iterator i=i0; i!=i1; ++i) {      
      ++sample_counter_;
      for (int j=0; j<3; ++j)
        gf_[j].update(*i);

      if (sample_counter_ == period_) {
        sample_counter_ = 0;
        const double alpha=0.15;
        for (int j=0; j<3; ++j) {
          buf_[j][buf_counter_] = (1-alpha)*buf_[j][buf_counter_] + alpha*std::abs(gf_[j].x());
          gf_[j].reset();
        }

        ++buf_counter_;
        if (buf_counter_ == buf_size_) {
          buf_counter_ = 0;
          int offset[3];
          for (int j=0; j<3; ++j) {
//             std::cout << "buf[" << j << "]  " << buf_[j] << std::endl;
            offset[j] = check_lock(buf_[j], sig_[j], bkgd_[j]);
            std::cout << "buf[" << j << "]  " << offset[j] << " (" <<
              sig_[j][0] << "," << 
              sig_[j][1] << "," << 
              sig_[j][2] << "," << 
              sig_[j][3] << "," << 
              sig_[j][4] << "," << 
              sig_[j][5] << ")" << std::endl;
          }
          if (offset[0] == offset[1] &&
              offset[0] == offset[2]) {
            state_ = LOCKED;
            counterPhase_ = -offset[0] * sp->sample_rate_Hz() * dt_;
          }
        }
      }    

      if (state_ == LOCKED) {
        ++counterPhase_;
        for (int j=0; j<3; ++j)
          gfPhase_[j].update(*i);

        if (counterPhase_ == periodPhase_) {
          for (int j=0; j<3; ++j) {
            phases_[j][counterSlot_] = pmPi(std::arg(gfPhase_[j].x()) - counterCarrier_*2*M_PI*df[j]*3.6);
            std::cout << "j=" << j
                      << " slot="  << counterSlot_
                      << " phase= " << phases_[j][counterSlot_] << " " << std::abs(gfPhase_[j].x())
                      << std::endl;
          }
          ++counterSlot_;

          if (counterSlot_ == 6) {
            counterSlot_ = 0;
            for (int j=0; j<3; ++j) {
              const time_duration
                dt(0,0,0, std::distance(i0, i)*time_duration::ticks_per_second()/sp->sample_rate_Hz());
              result_alpha::sptr rp = result_alpha::make("ALPHA_"+names[j],sp->approx_ptime()+dt, bkgd_[j], sig_[j], phases_[j]);
              std::cout << rp->approx_ptime() << " " << rp->name() << " ";
              rp->dump_data(std::cout);
              std::cout << std::endl;
            }
            // TBD
          }
        }
        if (counterPhase_ == periodSlot_) {
          counterCarrier_ += 1;
          counterCarrier_ %= 7;
          counterPhase_ = 0;
          for (int j=0; j<3; ++j)
            gfPhase_[j].reset();
        }

      }

    }      
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
    const int len_0 = int(0.5+0.4/dt_);
    const int len_2 = int(0.5+0.6/dt_);
    
    const int n = buf.size();
    std::vector<float> corr(buf.size(), 0);
    for (int i=0; i<buf.size(); ++i)
      for (int j=0; j<buf.size(); ++j)
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
      const bool is_bkgd((j%len_2) >= len_0);
      const bool is_sig ((j%len_2) <  len_0);
      n_bkgd         += is_bkgd;
      bkgd           += is_bkgd * buf[i];
      n_sig[j/len_2] += is_sig;
      sig[j/len_2]   += is_sig  * buf[i];
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
  size_t        period_;
  double        dt_;
  goertzel_type gf_[3];  
  size_t        buf_counter_;
  size_t        buf_size_;
  std::vector<float> buf_[3];

  size_t        periodPhase_;
  size_t        periodSlot_;     // [0-5]
  int           counterPhase_;
  int           counterSlot_;
  goertzel_type gfPhase_[3];  
  int           counterCarrier_; // [0-7]

  float         bkgd_[3];
  float         sig_[3][6];
  float         phases_[3][6];

} ;
