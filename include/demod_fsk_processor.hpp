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
#ifndef _DEMOD_FSK_PROCESSOR_HPP_cm131117_
#define _DEMOD_FSK_PROCESSOR_HPP_cm131117_

#include <complex>
#include <iostream>
#include <sstream>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "demod/early_late_synch.hpp"
#include "decode/efr.hpp"
#include "filter/iir.hpp"
#include "filter/iir_design.hpp"
#include "filter/fir_filter.hpp"
#include "FFT.hpp"
#include "FFTProcessor/Filter.hpp"
#include "logging.hpp"
#include "processor.hpp"
#include "Spectrum.hpp"

/*! \addtogroup processors
 *  @{
 * \addtogroup demod_fsk demod_fsk
 * FSK demodulation
 *
 * @{
 */

/// FSK demodulation processor
class demod_fsk_processor : public processor::base_iq {
public:
  typedef boost::shared_ptr<demod_fsk_processor> sptr;
  typedef std::complex<double> complex_type;

  class result_data : public processor::result_base {
  public:
    typedef boost::shared_ptr<result_data> sptr;
    typedef std::vector<unsigned char> data_vector_type;
    typedef data_vector_type::iterator iterator;
    typedef data_vector_type::const_iterator const_iterator;

    virtual ~result_data() {}
    static sptr make(std::string name,
                     ptime  t,
                     double fc_Hz,
                     double fs_Hz,
                     double baud) {
      return sptr(new result_data(name, t, fc_Hz, fs_Hz, baud));
    }

    double fc_Hz()   const { return fc_Hz_; }
    double fs_Hz()   const { return fs_Hz_; }
    double baud()    const { return baud_; }

    size_t size() const { return data_vec_.size(); }
    const_iterator begin() const { return data_vec_.begin(); }
    const_iterator end() const { return data_vec_.end(); }

    void clear() { data_vec_.clear(); }
    void push_back(unsigned char b) { data_vec_.push_back(b); }

    virtual std::ostream& dump_header(std::ostream& os) const {
      return os
        << "# fc[Hz] = " << boost::format("%15.8f") % fc_Hz() << "\n"
        << "# fs[Hz] = " << boost::format("%7.3f")  % fs_Hz() << "\n"
        << "# baud[Hz] = " << boost::format("%7.0f")  % baud() << "\n"
        << "# Time_UTC data ";
    }
    // f= @(bb) [bb(:,10) bb(:,9) bb(:,8)-160 bb(:,7) bb(:,6) bitshift(bb(:,5),-2) bitshift(bb(:,1),-4), bitand(bb(:,1),15)]

    virtual std::ostream& dump_data(std::ostream& os) const {
      os << boost::format("%02X %02X%02X ") % int(data_vec_[0]) % int(data_vec_[1]) % int(data_vec_[2]);
      for (const_iterator i(begin()+3), iend(end()); i!=iend; ++i)
        os << boost::format("%02X") % int(*i);

      static const char* weekdays[] = {
        "___",
        "Mon",
        "Tue",
        "Wed",
        "Thu",
        "Fri",
        "Sat",
        "Sun"
      };
      static const char* time_zones[] = {
        "MEZ",
        "MESZ"
      };

      if (data_vec_[1] == 0 &&
          data_vec_[2] == 0 &&
          data_vec_[3] == 0) { // time telegram
        os <<
          boost::format(" T: 20%02d-%02d-%02d (%s) %02d:%02d:%02d %s")
          % int(data_vec_[9] & 0x7F)
          % int(data_vec_[8] & 0x0F)
          % int(data_vec_[7] & 0x1F)
          % weekdays[data_vec_[7]>>5]
          % int(data_vec_[6] & 0x1F)
          % int(data_vec_[5] & 0x3F)
          % int(data_vec_[4]>>2)
          % time_zones[(data_vec_[6]&0x80) == 0x80];
      }

      if (data_vec_[1] == 0xFF &&
          data_vec_[2] == 0xFF) {
        os << " \'";
        std::copy(data_vec_.begin()+3, data_vec_.end(), std::ostream_iterator<char>(os, ""));
        os << "\'";
      }
      return os;
    }

  protected:
  private:
    result_data(std::string name,
                ptime  t,
                double fc_Hz,
                double fs_Hz,
                double baud)
      : result_base(name, t)
      , fc_Hz_(fc_Hz)
      , fs_Hz_(fs_Hz)
      , baud_(baud) {}

    const double     fc_Hz_;
    const double     fs_Hz_;
    const double     baud_;
    data_vector_type data_vec_;
  } ;

  typedef boost::posix_time::time_duration time_duration;

  typedef filter::iir<double, complex_type>         iir_type;
  typedef filter::iir_lowpass_1pole<double, double> iir_lp_type;
  typedef filter::integrator_modulo<double>         osc_phase_type;

  demod_fsk_processor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , name_(config.get<std::string>("<xmlattr>.name"))
    , fc_Hz_(config.get<double>("<xmlattr>.fc_Hz"))
    , fc_shifted_Hz_(0.0)
    , fs_Hz_(config.get<double>("<xmlattr>.fs_Hz"))
    , baud_(config.get<double>("<xmlattr>.baud"))
    , is_initialized_(false)
    , period_(1)
    , sample_counter_(0)
    , filter_(401, 0.02*baud_/200.)
    , dc_factor_(1)
    , osc_phase_mark_(2*M_PI)
    , osc_phase_space_(2*M_PI)
    , iir_filter_order_(config.get<size_t>("<xmlattr>.filterOrder"))
    , iir_mark_(1+iir_filter_order_)
    , iir_space_(1+iir_filter_order_)
    , iir_strength_(60., 1000.)
    , iir_startup_counter_(0)
    , early_late_synch_(5)
  {}

  virtual ~demod_fsk_processor() {}

  double fc_Hz() const { return fc_Hz_; }
  double fc_shifted_Hz() const { return fc_shifted_Hz_; }
  double fs_Hz() const { return fs_Hz_; }
  double baud()  const { return baud_; }

  void process_iq(service::sptr sp,
                  const_iterator i0,
                  const_iterator i1) {
    const double offset_ppb(sp->offset_ppb());
    const double offset_Hz(fc_Hz()*offset_ppb*1e-9);
#if 0
    std::cout << "process_iq nS=" << std::distance(i0, i1)
              << " " << sp->id()
              << " " << sp->approx_ptime()
              << " " << sp->sample_rate_Hz()
              << " " << sp->center_frequency_Hz()
              << " " << sp->offset_ppb()
              << " length=" << int(std::distance(i0, i1))
              << std::endl;
#endif
    if (not is_initialized_) {
      // measurements per bit
      period_ = size_t(0.5+sp->sample_rate_Hz()/baud()/10);

#if 0
      std::cout << "SHIFTs ({f_shift0,f_shift,fc}_Hz): " << f_shift0_Hz << " " << f_shift_Hz << " " << fc_Hz() << std::endl;
#endif
      // normalized mark frequency
      double k_mark ((fc_Hz() - fs_Hz() - sp->center_frequency_Hz()) / sp->sample_rate_Hz());
      // normalized space frequency
      double k_space((fc_Hz() + fs_Hz() - sp->center_frequency_Hz()) / sp->sample_rate_Hz());

      // filter +- baud()
      filter_.design(401, 2*baud()/sp->sample_rate_Hz());

      const double f_shift0_Hz(fc_Hz() - sp->center_frequency_Hz());
      double f_shift_Hz(filter_.shift(f_shift0_Hz/sp->sample_rate_Hz())*sp->sample_rate_Hz());
      fc_shifted_Hz_ = fc_Hz_ - f_shift_Hz;

      // downconversion to 5*baud Hz
      dc_factor_ = size_t(0.5+sp->sample_rate_Hz()/(5.*baud()));

      // iir filters for mark and space with cutoff +- fs_Hz
      filter::iir_design_lowpass iird(iir_filter_order_);
      iird.design(iir_filter_order_, 2.*fs_Hz()/sp->sample_rate_Hz()*dc_factor_);
      auto const a(iird.a());
      auto const b(iird.b());
      iir_mark_.init(a, b);
      iir_space_.init(a, b);

      iir_strength_.init(0.25, double(sp->sample_rate_Hz()) / dc_factor_);

      early_late_synch_ = demod::early_late_synch(double(sp->sample_rate_Hz()) / dc_factor_ / baud());

      std::cout << "k_mark,space,period= " << k_mark << " " << k_space << " " << period_ << " "
                << sp->sample_rate_Hz() << " "
                << sp->center_frequency_Hz() << " "
                << early_late_synch_.period() << " fc_Hz= " << fc_shifted_Hz() << std::endl;
#if 0
      std::cout << "a= "; std::copy(a.begin(), a.end(), std::ostream_iterator<double>(std::cout, ", "));
      std::cout << std::endl;
      std::cout << "b= "; std::copy(b.begin(), b.end(), std::ostream_iterator<double>(std::cout, ", "));
      std::cout << std::endl;
#endif
      if (not decoder_)
        decoder_ = decode::efr::make();

      is_initialized_ = true;
    }

    const double fs(double(sp->sample_rate_Hz()) / dc_factor_);
    const double f_mark ((fc_shifted_Hz() + fs_Hz() - sp->center_frequency_Hz()+offset_Hz)/fs);
    const double f_space((fc_shifted_Hz() - fs_Hz() - sp->center_frequency_Hz()+offset_Hz)/fs);
    for (const_iterator i(i0); i!=i1; ++i, ++sample_counter_) {
      if ((sample_counter_ % dc_factor_) != 0) {
        filter_.insert(*i);
      } else {
        const complex_type sample(filter_.process(*i));
        sample_counter_ = 0;
        osc_phase_mark_.process (2*M_PI*f_mark);
        osc_phase_space_.process(2*M_PI*f_space);
        const double sig_mark (std::abs(iir_mark_.process (sample*std::exp(complex_type(0, osc_phase_mark_.get())))));
        const double sig_space(std::abs(iir_space_.process(sample*std::exp(complex_type(0, osc_phase_space_.get())))));
        if (iir_startup_counter_ < size_t(0.5+3/baud()*fs)) {
          iir_strength_.reset(sig_mark + sig_space);
          ++iir_startup_counter_;
        } else {
          iir_strength_.process(sig_mark + sig_space);
          early_late_synch_.insert_signal((sig_mark - sig_space)/std::min(1e-10, iir_strength_.get()));
#if 0
          std::cout << "S "
                    << sig_mark << " " << sig_space << " "
                    << iir_strength_.get() << " "
                    << early_late_synch_.current_bit() << " " << early_late_synch_.bit_valid()
                    << "\n";
#endif
          if (early_late_synch_.bit_valid()) {
            decoder_->push_back(not early_late_synch_.current_bit());
            if (decoder_->data_ok()) {
              // 4 start bytes + data + checksum + end byte
              const size_t length(6+decoder_->data().size());
              const double dt_sec(-1.5*early_late_synch_.period() / fs);
              const time_duration
                dt(0,0,0, boost::int64_t(0.5 + (std::distance(i0, i)/double(sp->sample_rate_Hz()) - 11.*length/baud_ + dt_sec)*time_duration::ticks_per_second()));
#if 0
              std::cout << "EFR: len,dt,fs,dt_sec,dt_len_sec = "
                        << length << " "
                        << std::distance(i0, i)/double(sp->sample_rate_Hz()) << " "
                        << fs << " "
                        << dt_sec << " "
                        << 11.*length/baud_ << " "
                        << std::endl;
#endif
              result_data::sptr result_data =
                result_data::make(name_ + "_data", sp->approx_ptime()+dt, fc_Hz(), fs_Hz(), baud());
              for (auto const& d : decoder_->data())
                result_data->push_back(d);
#if 0
              std::cout << "push_back: " << result_data->to_string() << std::endl;
#endif
              sp->put_result(result_data);
            }
          }
        }
      }
    }

  }

  // virtual void dump(processor::result_base::sptr) {
  //   // to be overwritten in a derived class
  // }

protected:
private:
  const std::string name_;
  const double      fc_Hz_;         // center frequency
  double            fc_shifted_Hz_; // shifter center frequency
  const double      fs_Hz_;         // shift frequency
  const double      baud_;
  bool              is_initialized_;
  size_t            period_;
  size_t            sample_counter_;
  fir_filter        filter_;
  size_t            dc_factor_;    // down conversion factor
  osc_phase_type    osc_phase_mark_;
  osc_phase_type    osc_phase_space_;
  size_t            iir_filter_order_;
  iir_type          iir_mark_;
  iir_type          iir_space_;
  iir_lp_type       iir_strength_;
  size_t            iir_startup_counter_;
  demod::early_late_synch  early_late_synch_;
  decode::efr::sptr decoder_;
} ;
/// @}
/// @}

#endif // _DEMOD_FSK_PROCESSOR_HPP_cm131117_
