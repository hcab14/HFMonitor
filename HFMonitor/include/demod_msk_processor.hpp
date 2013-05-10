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
#ifndef _DEMOD_MSK_PROCESSOR_HPP_cm130214_
#define _DEMOD_MSK_PROCESSOR_HPP_cm130214_

#include <complex>
#include <iostream>
#include <sstream>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "demod/msk.hpp"
#include "filter/fir.hpp"
#include "FFT.hpp"
#include "FFTProcessor/Filter.hpp"
#include "logging.hpp"
#include "processor.hpp"
#include "Spectrum.hpp"

class fir_filter {
public:
  typedef std::complex<double> complex_type;
  typedef filter::fir::lowpass<double> fir_type;

  fir_filter(size_t n, double f)
    : n_(n)
    , f_(f)
    , fir_design_(n)
    , b_(n, 0.)
    , phases_ (n, complex_type(1,0))
    , history_(n, complex_type(0,0)) {
    fir_design_.design(f, f/10);
    std::copy(fir_design_.coeff().begin(), fir_design_.coeff().end(), b_.begin());
  }

  double shift(double f0) { 
    long int shift(lround(f0*n_));
    std::cout << "shift= " << shift << std::endl;
    while (shift >= int(n_/2)) shift -= n_;
    while (shift < -int(n_/2)) shift += n_;
    f0 = double(shift)/double(n_);
    for (size_t i(0); i<n_; ++i) {
      phases_[i] = std::exp(complex_type(0., 2*M_PI*f0*i));
      std::cout << i << " " << phases_[i] << " " << b_[i] << std::endl;
    }
    return f0;
  }

  complex_type process(complex_type sample) {
    for (size_t i(n_-1); i>0; --i)
      history_[i] = history_[i-1];
    history_[0] = sample;
    complex_type result(0);
    for (size_t i(0); i<n_; ++i)
      result += history_[i] * phases_[i] * fir_design_.coeff()[i];
    return result;
  }

protected:
private:
  const size_t n_;
  const double f_;
  fir_type fir_design_;
  std::vector<double> b_;
  std::vector<complex_type> phases_; // for shift
  std::vector<complex_type> history_;
} ;

class demod_msk_processor : public processor::base_iq {
public:
  typedef boost::shared_ptr<demod_msk_processor> sptr;
  typedef std::complex<double> complex_type;
  
  class result : public processor::result_base {
  public:
    typedef boost::shared_ptr<result> sptr;
    
    virtual ~result() {}
    static sptr make(std::string name,
                     ptime  t,
                     double fc_Hz,
                     double fm_Hz,
                     double amplitude,
                     double sn_db,
                     double phase_rad) {
      return sptr(new result(name, t, fc_Hz, fm_Hz, amplitude, sn_db, phase_rad));
    }

    double fc_Hz()     const { return fc_Hz_; }
    double fm_Hz()     const { return fm_Hz_; }
    double amplitude() const { return amplitude_; }
    double sn_db()     const { return sn_db_; }
    double phase_rad() const { return phase_rad_; }

    virtual std::ostream& dump_header(std::ostream& os) const {
      return os
        << "# fc[Hz] = " << boost::format("%15.8f") % fc_Hz() << "\n"
        << "# fm[Hz] = " << boost::format("%7.3f")  % fm_Hz() << "\n"
        << "# Time_UTC Amplitude[dB] S/N[db] Phase[rad] ";
    }
    virtual std::ostream& dump_data(std::ostream& os) const {
      return os
        << boost::format("%7.2f") % amplitude() << " "
        << boost::format("%7.2f") % sn_db()     << " "
        << boost::format("%6.3f") % phase_rad();
    }
    
  protected:
  private:
    result(std::string name,
           ptime  t,
           double fc_Hz,
           double fm_Hz,
           double amplitude,
           double sn_db,
           double phase_rad)
      : result_base(name, t)
      , fc_Hz_(fc_Hz)
      , fm_Hz_(fm_Hz)
      , amplitude_(amplitude)
      , sn_db_(sn_db)
      , phase_rad_(phase_rad) {}
    
    const double fc_Hz_;
    const double fm_Hz_;
    const double amplitude_;
    const double sn_db_;
    const double phase_rad_;
  } ;
  
  typedef boost::posix_time::time_duration time_duration;

  demod_msk_processor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , fftw_(1024, FFTW_BACKWARD, FFTW_ESTIMATE)
    , name_(config.get<std::string>("<xmlattr>.name"))
    , fc_Hz_(config.get<double>("<xmlattr>.fc_Hz"))
    , fm_Hz_(config.get<double>("<xmlattr>.fm_Hz"))
    , dwl_Hz_(config.get<double>("<xmlattr>.dwl_Hz"))
    , period_Sec_(config.get<double>("<xmlattr>.period_Sec"))
    , min_SN_db_(config.get<double>("<xmlattr>.min_SN_db"))
    , max_offset_ppb_rms_(config.get<double>("<xmlattr>.max_offset_ppb_rms"))
    , filter_(401, 0.05*fm_Hz_/200.)
    , filter_amp_pm_    (  config.get<double>("<xmlattr>.ampl_lowpass_tc_Sec"), period_Sec_)
    , filter_amp_center_(5*config.get<double>("<xmlattr>.ampl_lowpass_tc_Sec"), period_Sec_)
    , phase_(0) {
    filter_plus_.add (Filter::LowPass<frequency_vector<double> >::make(1.0, 300));
    filter_minus_.add(Filter::LowPass<frequency_vector<double> >::make(1.0, 300));
  }
  
  ~demod_msk_processor() {}

  void process_iq(service::sptr sp,
                  const_iterator i0,
                  const_iterator i1) {
    const size_t length(std::distance(i0, i1));
    if (length != fftw_.size())
      fftw_.resize(length);

    std::cout << "demod_msk_processor::process_iq " << sp->stream_name()
              << " sample_rate_Hz=" << sp->sample_rate_Hz()
              << " center_frequency_Hz=" << sp->center_frequency_Hz()
              << " offset_ppb= " << sp->offset_ppb()
              << " offset_ppb_rms= " << sp->offset_ppb_rms()
              << " fc_Hz=" << fc_Hz_
              << " fm_Hz=" << fm_Hz_
              << " max_offset_ppb_rms=" << max_offset_ppb_rms_
              << std::endl;
    const double offset_ppb(sp->offset_ppb());
    const double offset_Hz(fc_Hz_*offset_ppb*1e-9);

    if (sp->offset_ppb_rms() > max_offset_ppb_rms_) {
      LOG_INFO(str(boost::format("offset_ppb = %.2f > %.2f is too big") % sp->offset_ppb_rms() % max_offset_ppb_rms_));
      return;
    }

    // set up a new filter
    bool is_first_call(false);
    if (not demod_msk_) {
      std::cout << "msk: " << name_ << std::endl;
      demod_msk_ = demod::msk::make(sp->sample_rate_Hz(),
                                    -(fc_Hz_ - sp->center_frequency_Hz()) + offset_Hz,
                                    0.5*fm_Hz_,
                                    dwl_Hz_, period_Sec_);
//       filter_.shift((fc_Hz_ - sp->center_frequency_Hz())/sp->sample_rate_Hz());
      filter_amp_pm_.reset(-1);
      filter_amp_center_.reset(-1);
      is_first_call= true;
    }

    demod_msk_->update_ppb(sp->offset_ppb(),
                           -(fc_Hz_ - sp->center_frequency_Hz()) + offset_Hz,
                           0.5*fm_Hz_);

    std::vector<complex_type> samples2(length, complex_type(0));
    std::vector<complex_type>::iterator j(samples2.begin());
    for (const_iterator i(i0); i!=i1; ++i, ++j) {
      const complex_type sample(filter_.process(*i));
      *j = (sample*sample);
      demod_msk_->process(sample);
      if (not is_first_call && demod_msk_->updated()) {
        const double amplitude_minus (10*std::log10(std::abs(demod_msk_->gf_minus().x())));
        const double amplitude_center(10*std::log10(std::abs(demod_msk_->gf_center().x())));
        const double amplitude_plus  (10*std::log10(std::abs(demod_msk_->gf_plus().x())));

        if (filter_amp_pm_.get() < 0) {
          filter_amp_pm_.reset(0.5*(amplitude_plus+amplitude_minus));
          filter_amp_center_.reset(amplitude_center);
        } else {
          filter_amp_pm_.process(0.5*(amplitude_plus+amplitude_minus));
          filter_amp_center_.process(amplitude_center);
        }
        const double amplitude(filter_amp_pm_.get());
        const double sn_db(amplitude - filter_amp_center_.get());

        if (sn_db < min_SN_db_) {
          continue;
          // demod_msk_->reset();
        }

        const double delta_phase_rad(demod_msk_->delta_phase_rad()
                                     + 2*M_PI*demod_msk_->period_sec() * (fc_Hz_ - sp->center_frequency_Hz() - offset_Hz));
        phase_ += delta_phase_rad;
        while (phase_ >= M_PI) phase_ -= 2*M_PI;
        while (phase_ < -M_PI) phase_ += 2*M_PI;
        std::cout << "XXX A+-0: " << amplitude << " " << sn_db << " " << sn_db+amplitude
                  << " P: " << 0.5*(demod_msk_->pll_plus().theta() + demod_msk_->pll_minus().theta())
                  << " DeltaF: " << demod_msk_->delta_phase_rad() /2/M_PI/demod_msk_->period_sec()
                  << " " << delta_phase_rad /2/M_PI/demod_msk_->period_sec()
                  << " " << phase_
                  << " " << fc_Hz_ - sp->center_frequency_Hz()
                  << std::endl;
        const time_duration
          dt(0,0,0, std::distance(i0, i)*time_duration::ticks_per_second()/sp->sample_rate_Hz());
        sp->put_result(result::make(name_, sp->approx_ptime()+dt, fc_Hz_, fm_Hz_, amplitude, sn_db, phase_));
//         dump(result::make(name_, sp->approx_ptime()+dt, fc_Hz_, fm_Hz_, amplitude, sn_db, phase_));
      }
    }
    fftw_.transformVector(samples2, FFT::WindowFunction::Blackman<double>());
    const FFTWSpectrum<double> s(fftw_, sp->sample_rate_Hz(), -2*(fc_Hz_ - sp->center_frequency_Hz()));

    double f_plus(0), f_minus(0), sn_plus(0), sn_minus(0);

    std::cout << "ps: " << name_ << " " << length << " " << s.index2freq(length/2-1) << " " << s.index2freq(length/2) 
              << " minus " << (fc_Hz_ - sp->center_frequency_Hz()) << std::endl;
    {
      const frequency_vector<double> ps(-120., -45., s, std::abs<double>);
      if (filter_minus_.x().empty())
        filter_minus_.init(sp->approx_ptime(), ps);
      else
        filter_minus_.update(sp->approx_ptime(), ps);
      
      const frequency_vector<double>& psf(filter_minus_.x());
      frequency_vector<double>::const_iterator i_max;
      double max(-1);
      double sum(0);
      for (frequency_vector<double>::const_iterator i(psf.begin()); i!=psf.end(); ++i) {
        std::cout << "ps: " << name_ << boost::format("%8.3f %.2e") % i->first % i->second << std::endl;
        sum += i->second;
        if (i->second > max) {
          max = i->second;
          i_max = i;
        }
      }
      double sum_w(0), sum_wx(0);
      for (frequency_vector<double>::const_iterator i(i_max-2); i!=(i_max+3) && i!=psf.end(); ++i) {
        sum_w += i->second;
        sum_wx += i->first * i->second;
      }
      f_minus  = sum_wx/sum_w;
      sn_minus = i_max->second/sum*psf.size();
    }
    {
      const frequency_vector<double> ps(45., 120., s, std::abs<double>);
      if (filter_plus_.x().empty())
        filter_plus_.init(sp->approx_ptime(), ps);
      else
        filter_plus_.update(sp->approx_ptime(), ps);

      const frequency_vector<double>& psf(filter_plus_.x());
      frequency_vector<double>::const_iterator i_max;
      double max(-1);
      double sum(0);
      for (frequency_vector<double>::const_iterator i(psf.begin()); i!=psf.end(); ++i) {
        std::cout << "ps: " << name_ << boost::format("%8.3f %.2e") % i->first % i->second << std::endl;
        sum += i->second;
        if (i->second > max) {
          max = i->second;
          i_max = i;
        }
      }
      double sum_w(0), sum_wx(0);
      for (frequency_vector<double>::const_iterator i(i_max-2); i!=(i_max+3) && i!=psf.end(); ++i) {
        sum_w += i->second;
        sum_wx += i->first * i->second;
      }
      f_plus  = sum_wx/sum_w;
      sn_plus = i_max->second/sum*psf.size();
    }
    std::cout << "ps: " << name_
              << boost::format(" f_minus,plus: %8.3f %8.3f sn_minus,plus: %5.2f %5.2f delta_f: %8.3f baud: %8.3f")
      % f_minus % f_plus
      % sn_minus % sn_plus
      % (0.5*(f_minus+f_plus))
      % (f_plus-f_minus)
              << std::endl;
  }

  virtual void dump(result::sptr) {
    // to be overwritten in a derived class
  }

protected:
private:
  FFT::FFTWTransform<double> fftw_;
  Filter::Cascaded<frequency_vector<double> > filter_plus_;
  Filter::Cascaded<frequency_vector<double> > filter_minus_;
  const std::string name_;
  const double      fc_Hz_;     // center frequency
  const double      fm_Hz_;     // modulation frequency
  const double      dwl_Hz_;
  const double      period_Sec_;
  const double      min_SN_db_;
  const double      max_offset_ppb_rms_;
  demod::msk::sptr  demod_msk_;
  fir_filter        filter_;
  filter::iir_lowpass_1pole<double, double> filter_amp_pm_;
  filter::iir_lowpass_1pole<double, double> filter_amp_center_;
  double            phase_;
} ;

#endif // _DEMOD_MSK_PROCESSOR_HPP_cm130214_
