// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2011 Christoph Mayer
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
#ifndef _portaudio_fft_hpp_cm110902_
#define _portaudio_fft_hpp_cm110902_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>

#include "util.hpp"
#include "portaudio.hpp"
#include "logging.hpp"

#include "FFT.hpp"
#include "mat_spectrum_saver.hpp"

// portaudio callback class
class pa_fft : public portaudio::stream_callback::process_base {
public:
  typedef boost::shared_ptr<pa_fft> sptr;
  typedef float FFTType; 
  typedef std::vector<std::complex<FFTType> > fft_vector;
  typedef boost::property_tree::ptree ptree;

  typedef boost::posix_time::ptime ptime;
  typedef boost::posix_time::time_duration time_duration;

  pa_fft(const ptree& config, 
         const ptree& config_saver, 
         double sample_rate)
    : center_frequency_(config.get<double>("<xmlattr>.centerFrequency_Hz"))
    , sample_rate_(sample_rate)
    , frames_(sample_rate_/config.get<double>("<xmlattr>.frequencyResolution_Hz"))
    , num_average_(config.get<size_t>("<xmlattr>.numAverage"))
    , fft_(frames_, FFTW_FORWARD, FFTW_ESTIMATE)
    , ps_(config.get<double>("<xmlattr>.frequencyMin_Hz", center_frequency_-0.5*47/48*sample_rate_),
          config.get<double>("<xmlattr>.frequencyMax_Hz", center_frequency_+0.5*47/48*sample_rate_), 
          sample_rate_, 0)
    , ps_sum_(ps_)
    , counter_(0)
    , saver_(config, config_saver) {}

  virtual ~pa_fft() {}

  struct oct_values {
    oct_values(size_t cols)
      : freq(0,cols)
      , spec(0,cols)
      , time(0,1) {}
    Matrix freq;
    Matrix spec;
    Matrix time;
  } ;

  static sptr make(const ptree& config, const ptree& config_saver, double sample_rate) {
    return sptr(new pa_fft(config, config_saver, sample_rate));
  }

  double center_frequency() const { return center_frequency_; }
  size_t frames() const { return frames_; }

  virtual void init() {
    counter_    = 0;
  }

  virtual int process(const void *input_buffer, 
                      void *output_buffer,
                      unsigned long frames_per_buffer,
                      portaudio::callback_info::sptr info) {
    // std::cout << "pa_fft callback called " << frames_per_buffer << " " << info->to_string() 
    //           << " " << info->input_buffer_adc_time() - info->current_time() << std::endl;
    fft_vector::const_iterator begin(static_cast<const fft_vector::value_type*>(input_buffer));
    fft_vector::const_iterator end(begin+frames_per_buffer);
    fft_.transformRange(begin, end, FFT::WindowFunction::Blackman<FFTType>());
    FFTWSpectrum<FFTType> fs(fft_, sample_rate_, center_frequency_);
    ps_.fill(fs, std::abs<double>);
    if (counter_++ == 0) 
      ps_sum_ = ps_;
    else 
      ps_sum_ += ps_;
    if (counter_ == num_average_) {
      ps_sum_ /= num_average_;
      const ptime now(boost::posix_time::microsec_clock::universal_time());
      const time_duration dt(0, 0, 0,
                             num_average_/2 * frames_per_buffer/sample_rate_*time_duration::ticks_per_second());
      saver_.save_spectrum(ps_sum_, now-dt);
    }
    return (counter_ == num_average_) ? paAbort : paContinue;
  }
protected:
private:
  const double center_frequency_;
  const double sample_rate_;
  const size_t frames_;
  const size_t num_average_;
  FFT::FFTWTransform<FFTType> fft_;
  frequency_vector<FFTType> ps_;
  frequency_vector<FFTType> ps_sum_;
  size_t             counter_;
  mat_spectrum_saver saver_;
} ;

#endif // _portaudio_fft_hpp_cm110902_
