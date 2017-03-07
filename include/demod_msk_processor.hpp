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
#ifndef _DEMOD_MSK_PROCESSOR_HPP_cm130214_
#define _DEMOD_MSK_PROCESSOR_HPP_cm130214_

#include <cmath>
#include <complex>
#include <iostream>
#include <sstream>
#include <vector>
#include <bitset>

#include <boost/property_tree/ptree.hpp>

#include "demod/msk.hpp"
#include "decode/rtcm2.hpp"
#include "FFT.hpp"
#include "FFTProcessor/Filter.hpp"
#include "filter/fir_filter.hpp"
#include "logging.hpp"
#include "processor.hpp"
#include "Spectrum.hpp"
#include "aligned_vector.hpp"

#include <complex>
#include <vector>

/*! \addtogroup processors
 *  @{
 * \addtogroup demod_msk demod_msk
 * MSK demodulation
 *
 * @{
 */

/// MSK demodulation processor
class demod_msk_processor : public processor::base_iq {
public:
  typedef boost::shared_ptr<demod_msk_processor> sptr;
  typedef std::complex<float> complex_type;

  class result_phase : public processor::result_base {
  public:
    typedef boost::shared_ptr<result_phase> sptr;

    virtual ~result_phase() {}
    static sptr make(std::string name,
                     ptime  t,
                     double fc_Hz,
                     double fm_Hz,
                     double amplitude,
                     double sn_db,
                     double phase_rad) {
      return sptr(new result_phase(name, t, fc_Hz, fm_Hz, amplitude, sn_db, phase_rad));
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
    result_phase(std::string name,
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

    const bit_vector_type& bits() const { return bitvec_; }

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

  class result_rtcm2 : public processor::result_base {
  public:
    typedef boost::shared_ptr<result_rtcm2> sptr;
    virtual ~result_rtcm2() {}

    static sptr make(std::string name, ptime t, std::string line) {
      return sptr(new result_rtcm2(name, t, line));
    }

    virtual std::ostream& dump_header(std::ostream& os) const {
      return os
        << "# name = " << name() << "\n"
        << "# Time_UTC message num msg_type z_count seq num_frames [msg] ";
    }
    virtual std::ostream& dump_data(std::ostream& os) const {
      return os << line_;
    }

  protected:
  private:
    result_rtcm2(std::string name, ptime t, std::string line)
      : result_base(name, t)
      , line_(line) {}

    std::string line_;
  } ;



  typedef boost::posix_time::time_duration time_duration;
  typedef FFT::FFTWTransform<float> fft_type;

  demod_msk_processor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , fftw_(1024, FFTW_BACKWARD, FFTW_ESTIMATE)
    , name_(config.get<std::string>("<xmlattr>.name"))
    , fc_Hz_(config.get<double>("<xmlattr>.fc_Hz"))
    , fm_Hz_(config.get<double>("<xmlattr>.fm_Hz"))
    , record_bits_(config.get<bool>("<xmlattr>.recordBits", false))
    , decode_rtcm_(config.get<bool>("<xmlattr>.decodeRTCM", false))
    , dwl_Hz_(config.get<double>("<xmlattr>.dwl_Hz"))
    , period_Sec_(config.get<double>("<xmlattr>.period_Sec"))
    , min_SN_db_(config.get<double>("<xmlattr>.min_SN_db"))
    , max_offset_ppb_rms_(config.get<double>("<xmlattr>.max_offset_ppb_rms"))
    , downscale_factor_(1)
    , delta_f_Hz_(0)
    , filter_(401, 0.05*fm_Hz_/200.)
    , filter_amp_pm_    (  config.get<double>("<xmlattr>.ampl_lowpass_tc_Sec"), period_Sec_)
    , filter_amp_center_(5*config.get<double>("<xmlattr>.ampl_lowpass_tc_Sec"), period_Sec_)
    , phase_(0)
    , signal_present_(0) {
    filter_plus_.add (Filter::LowPass<frequency_vector<double> >::make(1.0, 300));
    filter_minus_.add(Filter::LowPass<frequency_vector<double> >::make(1.0, 300));
  }

  virtual ~demod_msk_processor() {}

  double fc_Hz() const { return fc_Hz_; }
  double fm_Hz() const { return fm_Hz_; }

  double delta_f_Hz() const { return delta_f_Hz_; }

  void process_iq(service::sptr sp,
                  const_iterator i0,
                  const_iterator i1) {

    if (sp->offset_ppb_rms() > max_offset_ppb_rms_) {
      LOG_INFO(str(boost::format("offset_ppb = %.2f > %.2f is too big") % sp->offset_ppb_rms() % max_offset_ppb_rms_));
      return;
    }

    // offset due to the perseus oscillator
    const double offset_ppb(sp->offset_ppb());
    double offset_Hz((fc_Hz() + delta_f_Hz()) * offset_ppb*1e-9);

    // set up a new filter
    bool is_first_call(demod_msk_ == 0);

    if (is_first_call) {
      // FIR filter design:
      //   0.05 = 250/sp->sample_rate_Hz() for 200 baud
      filter_.design(1601, 1.25*fm_Hz_/sp->sample_rate_Hz());

      // difference between filter center frequency and samples center frequency
      const double f_shift0_Hz(fc_Hz() - sp->center_frequency_Hz());
      LOG_INFO(str(boost::format("%s f_shift0_Hz=%f") % name_ % f_shift0_Hz));

      // shift of the FIR filter including quantization
      const double f_shift_Hz(filter_.shift(f_shift0_Hz/sp->sample_rate_Hz())*sp->sample_rate_Hz());
      LOG_INFO(str(boost::format("%s f_shift_Hz=%f") % name_ % f_shift_Hz));

      // difference between set center frequency and true center frequency TBD
      delta_f_Hz_ = sp->center_frequency_Hz() + f_shift_Hz - fc_Hz();
      LOG_INFO(str(boost::format("%s delta_f_Hz=%f") % name_ % delta_f_Hz()));

      // update the offset due to the perseus oscillator
      offset_Hz = (fc_Hz() + delta_f_Hz()) * offset_ppb*1e-9;
      LOG_INFO(str(boost::format("%s offset_Hz=%f fc_Hz=%f offset_ppb=%f") % name_ % offset_Hz % fc_Hz() % offset_ppb));

      // compute the downscaling factor
      const size_t length(std::distance(i0, i1));
      downscale_factor_ = std::max(1, int(0.25*sp->sample_rate_Hz()/(1.25*fm_Hz_)));
      for (; downscale_factor_> 1 && std::fmod(double(length), double(downscale_factor_))!=0.0; --downscale_factor_)
        ;
      // downscale_factor_ = 1;
      LOG_INFO(str(boost::format("%s downscale_factor=%d") % name_ % downscale_factor_));

      demod_msk_ = demod::msk::make(sp->sample_rate_Hz()/downscale_factor_, // sampling frequency
                                    delta_f_Hz() + offset_Hz,               // center frequency relative to freq.-shifted FIR filter outpu
                                    0.5*fm_Hz(),                            // baud/2
                                    dwl_Hz_, period_Sec_);

      filter_amp_pm_.reset(-1);
      filter_amp_center_.reset(-1);
      signal_present_ = 0;
    }


    const size_t length(std::distance(i0, i1)/downscale_factor_);
    if (length != fftw_.size())
      fftw_.resize(length);

    demod_msk_->update_ppb(offset_ppb,
                           delta_f_Hz() + offset_Hz,
                           0.5*fm_Hz());

    // filter input
    aligned_vector<complex_type>
      samples(length, complex_type(0)),
      samples2(length, complex_type(0));
    aligned_vector<complex_type>::iterator
      j(samples.begin()),
      k(samples2.begin());
    for (const_iterator i(i0); i!=i1; ++i) {
      filter_.insert(*i);
      if (downscale_factor_ == 1 ||
          (std::distance(i0, i) % downscale_factor_) == 0) {
        const complex_type sample(filter_.process());
        *j++ = sample;
        *k++ = sample*sample;
      }
    }

#if 0
    {
      fftw_.transformRange(samples.begin(), samples.end(), FFT::WindowFunction::Blackman<float>(samples.size()));
      const FFTSpectrum<fft_type> s(fftw_, sp->sample_rate_Hz()/downscale_factor_, -2.*delta_f_Hz());

      for (int i=0; i<s.size(); ++i)
        std::cout << "SPEC: " << " " << s.index2freq(i) << " " << std::abs(s[i]) << std::endl;

      for (int i=0; i<samples2.size(); ++i)
        std::cout << "SIG: " << samples[i].real() << " " << samples[i].imag() << std::endl;

    }
#endif
    double baud(0), delta_f(0), sn(0);
    const bool is_signal = (detect_msk(sp, samples2.begin(), samples2.end(), baud, delta_f, sn) &&
                            sn > min_SN_db_       &&
                            std::abs(delta_f) < 5 &&
                            (std::abs(baud-100) < 5 ||
                             std::abs(baud-200) < 5));

    size_t sample_counter = 0;
    for (const_iterator i(samples.begin()); i!=samples.end(); ++i, ++sample_counter) {
      const complex_type sample(*i);
      demod_msk_->process(sample);

      // amplitude and phase data
      if (not is_first_call && demod_msk_->phase_available()) {
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

        if (is_signal) {
          ++signal_present_;

          const double delta_phase_rad(demod_msk_->delta_phase_rad()
                                       - 2*M_PI*demod_msk_->period_sec() * (delta_f_Hz() + offset_Hz));
          phase_ += delta_phase_rad;
          while (phase_ >= M_PI) phase_ -= 2*M_PI;
          while (phase_ < -M_PI) phase_ += 2*M_PI;
          // std::cout << "XXX A+-0: " << amplitude << " " << sn_db << " " << sn_db+amplitude
          //           << " P: " << 0.5*(demod_msk_->pll_plus().theta() + demod_msk_->pll_minus().theta())
          //           << " DeltaF: " << demod_msk_->delta_phase_rad() /2/M_PI/demod_msk_->period_sec()
          //           << " " << delta_phase_rad /2/M_PI/demod_msk_->period_sec()
          //           << " " << phase_
          //           << " " << fc_Hz() - sp->center_frequency_Hz()
          //           << std::endl;
          const time_duration
            dt(0,0,0, sample_counter*time_duration::ticks_per_second()/sp->sample_rate_Hz()*downscale_factor_);
          result_phase::sptr r(result_phase::make(name_, sp->approx_ptime()+dt, fc_Hz(), fm_Hz_, amplitude, sn_db, phase_));
          sp->put_result(r);
        }
      }

      // collect bit data
      if (not is_first_call && demod_msk_->bit_available()) {
        const time_duration
          dt(0,0,0, sample_counter*time_duration::ticks_per_second()/sp->sample_rate_Hz()*downscale_factor_);
        if (not result_bits_) {
          result_bits_ = result_bits::make(name_+"_bits", sp->approx_ptime()+dt, fc_Hz(), fm_Hz());
        }
        result_bits_->push_back(demod_msk_->current_bit());
        if (decode_rtcm_) {
          const std::string line = rtcm_decoder_.decode(demod_msk_->current_bit());
          if (line != "") {
            sp->put_result(result_rtcm2::make(name_+"_rtcm2", sp->approx_ptime()+dt, line));
          }
        }
        if (result_bits_->size() == size_t(0.5+fm_Hz())) {
          result_bits_->set_quality(double(signal_present_));
          //           std::cout << name_ << " " << "quality= " << result_bits_->quality() << " " << signal_present_ << std::endl;

          if (record_bits_ && result_bits_->quality() > 0.5)
            sp->put_result(result_bits_);
          result_bits_.reset();
          signal_present_ = 0;
        }
      }
    }
  }

  // virtual void dump(processor::result_base::sptr) {
  //   // to be overwritten in a derived class
  // }

protected:
  bool detect_msk(service::sptr sp,
                  aligned_vector<complex_type>::const_iterator i0,
                  aligned_vector<complex_type>::const_iterator i1,
                  double &baud,
                  double &delta_f,
                  double &sn) {
    const size_t length(std::distance(i0, i1));
    fftw_.transformRange(i0, i1, FFT::WindowFunction::Blackman<float>(length));
    const FFTSpectrum<fft_type> s(fftw_, sp->sample_rate_Hz()/downscale_factor_, 2.*delta_f_Hz());
#if 0
    std::cout << "ps: " << name_ << " " << length << " " << s.index2freq(length/2-1) << " " << s.index2freq(length/2)
              << " minus " << fc_Hz() << " " <<  filter_.get_shift()*sp->sample_rate_Hz() << " " << sp->center_frequency_Hz() << " "
              << sp->sample_rate_Hz() << " " << downscale_factor_ << std::endl;
#endif
    const frequency_vector<double> ps_plus(-250., -40., s, std::abs<float>);
    if (filter_minus_.x().empty())
      filter_minus_.init(sp->approx_ptime(), ps_plus);
    else
      filter_minus_.update(sp->approx_ptime(), ps_plus);

    const frequency_vector<double> ps_minus(40., 250., s, std::abs<double>);
    if (filter_plus_.x().empty())
      filter_plus_.init(sp->approx_ptime(), ps_minus);
    else
      filter_plus_.update(sp->approx_ptime(), ps_minus);

    double f_minus(0), sn_minus(0);
    const bool found_minus = find_peak(filter_minus_.x(), f_minus, sn_minus);
    if (!found_minus) return false;

    double f_plus(0), sn_plus(0);
    const bool found_plus = find_peak(filter_plus_.x(), f_plus, sn_plus);
    if (!found_plus) return false;

    try {
#if 0
      const size_t ip[2] = {
        s.freq2index(f_minus),
        s.freq2index(f_plus)
      };
      for (int i=-3; i<4; ++i) {
        std::cout << "detect_msk: " << i << " "
                  << s.index2freq(i+ip[0]) << " "
                  << s.index2freq(i+ip[1]) << " "
                  << std::arg(fftw_.getBin(i+ip[0])) - std::arg(fftw_.getBin(i+ip[1])) << " "
                  << std::abs(fftw_.getBin(i+ip[0])) << " "
                  << std::abs(fftw_.getBin(i+ip[1])) << " "
                  << std::endl;
      }
#endif
      sn      = 0.5*(sn_minus + sn_plus);
      delta_f = 0.5* (f_plus + f_minus);
      baud    = f_plus - f_minus;
      // std::cout << "ps: " << delta_f << " " << sn << " " << baud << std::endl;
    } catch (const std::exception& e) {
      LOG_ERROR(e.what());
      return false;
    }
    return true;
  }

  bool find_peak(const frequency_vector<double>& psf, double &f, double &sn) const {
    frequency_vector<double>::const_iterator i_max;
    double max(-1);
    double sum(0);
    for (frequency_vector<double>::const_iterator i(psf.begin()), end(psf.end()); i!=end; ++i) {
      // std::cout << "find_peak ps: " << name_ << boost::format("%8.3f %.2e") % i->first % i->second << std::endl;
      sum += i->second;
      if (i->second > max) {
        max = i->second;
        i_max = i;
      }
    }
    double sum_w(0), sum_wx(0);
    for (frequency_vector<double>::const_iterator i(i_max-3); i!=(i_max+4) && i!=psf.end(); ++i) {
      if (i < psf.begin())
        continue;
      // std::cout << "ps: " << name_ << boost::format("%8.3f %.2e") % i->first % i->second << std::endl;
      sum_w += i->second;
      sum_wx += i->first * i->second;
    }
    f  = sum_wx/sum_w;
    sn = i_max->second/sum*psf.size();
    // std::cout << " --- " << f << " (" << sn << ") " << std::endl;
    return true;
  }
private:
  fft_type fftw_;
  Filter::Cascaded<frequency_vector<double> > filter_plus_;
  Filter::Cascaded<frequency_vector<double> > filter_minus_;
  const std::string name_;
  double            fc_Hz_;     // center frequency
  const double      fm_Hz_;     // baud
  const bool        record_bits_;
  const bool        decode_rtcm_;
  const double      dwl_Hz_;
  const double      period_Sec_;
  const double      min_SN_db_;
  const double      max_offset_ppb_rms_;
  int               downscale_factor_;
  double            delta_f_Hz_;
  demod::msk::sptr  demod_msk_;
  fir_filter        filter_;
  filter::iir_lowpass_1pole<double, double> filter_amp_pm_;
  filter::iir_lowpass_1pole<double, double> filter_amp_center_;
  double            phase_;
  size_t            signal_present_;
  result_bits::sptr result_bits_; // accumulates bits
  decode::rtcm2     rtcm_decoder_;
} ;
/// @}
/// @}
#endif // _DEMOD_MSK_PROCESSOR_HPP_cm130214_
