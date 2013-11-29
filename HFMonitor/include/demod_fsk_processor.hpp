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

class demod_fsk_processor : public processor::base_iq {
public:
  typedef boost::shared_ptr<demod_fsk_processor> sptr;
  typedef std::complex<double> complex_type;
  
  class result_bits : public processor::result_base {
  public:
    typedef boost::shared_ptr<result_bits> sptr;
    typedef std::vector<bool> bit_vector_type;
    typedef bit_vector_type::iterator iterator;
    typedef bit_vector_type::const_iterator const_iterator;

    virtual ~result_bits() {}
    static sptr make(std::string name,
                     ptime  t,
                     double fc_Hz,
                     double fm_Hz) {
      return sptr(new result_bits(name, t, fc_Hz, fm_Hz));
    }

    double fc_Hz()   const { return fc_Hz_; }
    double fm_Hz()   const { return fm_Hz_; }
    double quality() const { return quality_; }

    void  set_quality(double q) { quality_ = q; }

    size_t size() const { return bitvec_.size(); }
    const_iterator begin() const { return bitvec_.begin(); }
    const_iterator end() const { return bitvec_.end(); }

    void clear() { bitvec_.clear(); }
    void push_back(bool bit) { bitvec_.push_back(bit); }

    virtual std::ostream& dump_header(std::ostream& os) const {      
      return os
        << "# fc[Hz] = " << boost::format("%15.8f") % fc_Hz() << "\n"
        << "# fm[Hz] = " << boost::format("%7.3f")  % fm_Hz() << "\n"
        << "# Time_UTC quality[%] bits ";
    }
    virtual std::ostream& dump_data(std::ostream& os) const {
      os << boost::format("%3.0f") % (100.*quality()) << " ";
      for (const_iterator i(begin()); i!=end();) {
        unsigned char x(0);
        for (size_t j(0); j<4 && i!=end(); ++j, ++i)
          x |= (*i << (3-j));
        const char c(x<10 ? '0'+x : 'A'+x-10);
        os << c;
      }
      return os;
    }
    
  protected:
  private:
    result_bits(std::string name,
               ptime  t,
               double fc_Hz,
               double fm_Hz)
      : result_base(name, t)
      , fc_Hz_(fc_Hz)
      , fm_Hz_(fm_Hz)
      , quality_(0) {}
    
    const double    fc_Hz_;
    const double    fm_Hz_;
    double          quality_;
    bit_vector_type bitvec_;
  } ;
 
  typedef boost::posix_time::time_duration time_duration;
  
  typedef filter::iir<double, complex_type>         iir_type;
  typedef filter::iir_lowpass_1pole<double, double> iir_lp_type;
  typedef filter::integrator_modulo<double>         osc_phase_type;

  demod_fsk_processor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , name_(config.get<std::string>("<xmlattr>.name"))
    , fc_Hz_(config.get<double>("<xmlattr>.fc_Hz"))
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
  double fs_Hz() const { return fs_Hz_; }
  double baud()  const { return baud_; }

  void process_iq(service::sptr sp,
                  const_iterator i0,
                  const_iterator i1) {
    const double offset_ppb(sp->offset_ppb());
    const double offset_Hz(fc_Hz()*offset_ppb*1e-9);
    if (not is_initialized_) {
      // measurements per bit
      period_ = size_t(0.5+sp->sample_rate_Hz()/baud()/10);

      const double f_shift0_Hz(fc_Hz() - sp->center_frequency_Hz());
      double f_shift_Hz(filter_.shift(f_shift0_Hz/sp->sample_rate_Hz())*sp->sample_rate_Hz());
      fc_Hz_ -= f_shift_Hz;

      // normalized mark frequency
      double k_mark ((fc_Hz() - fs_Hz() - sp->center_frequency_Hz()) / sp->sample_rate_Hz());
      // normalized space frequency
      double k_space((fc_Hz() + fs_Hz() - sp->center_frequency_Hz()) / sp->sample_rate_Hz());

      // filter +- baud()
      filter_.design(401, 2*baud()/sp->sample_rate_Hz());

      // downconversion to 5*baud Hz
      dc_factor_ = size_t(0.5+sp->sample_rate_Hz()/(5*baud()));

      // iir filters for mark and space with cutoff +- fs_Hz
      filter::iir_design_lowpass iird(iir_filter_order_);
      iird.design(iir_filter_order_, 2*fs_Hz()/sp->sample_rate_Hz()*dc_factor_);
      const filter::iir_design_lowpass::vector_type a(iird.a());
      const filter::iir_design_lowpass::vector_type b(iird.b());
      iir_mark_.init(a, b);
      iir_space_.init(a, b);

      iir_strength_.init(0.25, sp->sample_rate_Hz() / dc_factor_);

      early_late_synch_ = demod::early_late_synch(sp->sample_rate_Hz() / dc_factor_ / baud());

      std::cout << "k_mark,space,period= " << k_mark << " " << k_space << " " << period_ << " " 
                << sp->sample_rate_Hz() << " " 
                << sp->center_frequency_Hz() << " "
                << early_late_synch_.period() << std::endl;
      std::cout << "a= "; std::copy(a.begin(), a.end(), std::ostream_iterator<double>(std::cout, ", "));
      std::cout << std::endl;
      std::cout << "b= "; std::copy(b.begin(), b.end(), std::ostream_iterator<double>(std::cout, ", "));
      std::cout << std::endl;

      if (not decoder_)
        decoder_ = decode::efr::make();

      is_initialized_ = true;
    }

    const double fs(sp->sample_rate_Hz() / dc_factor_);
    const double f_mark ((fc_Hz() + fs_Hz() - sp->center_frequency_Hz()+offset_Hz)/fs);
    const double f_space((fc_Hz() - fs_Hz() - sp->center_frequency_Hz()+offset_Hz)/fs);
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
          std::cout << "S " 
                    << sig_mark << " " << sig_space << " "
                    << iir_strength_.get() << " "
                    << early_late_synch_.current_bit() << " " << early_late_synch_.bit_valid()
                    << "\n";
          if (early_late_synch_.bit_valid())
            decoder_->push_back(not early_late_synch_.current_bit());
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
  double            fc_Hz_;     // center frequency
  const double      fs_Hz_;     // shift frequency
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
  result_bits::sptr result_bits_; // accumulates bits
} ;

#endif // _DEMOD_FSK_PROCESSOR_HPP_cm131117_
