// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_RESULT_FSK_STRENGTH_HPP_cm101118_
#define _FFT_RESULT_FSK_PEAK_STRENGTH_cm101118_

#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>

#include "Spectrum.hpp"
#include "FFTProcessor/Result.hpp"

namespace Result {
  class FFTResultFSKStrength : public Base {
  public:
    typedef boost::shared_ptr<FFTResultFSKStrength> Handle;
    typedef frequency_vector<double> PowerSpectrum;
    FFTResultFSKStrength(ptime time,
                         Result::SpectrumPeak::Handle peakRef,
                         Result::SpectrumPeak::Handle peakShift,
                         Result::SpectrumPowerInInterval::Handle noise)
      : Base("FFTResultFSKStrength", time)
      , peakRef_(peakRef) 
      , peakShift_(peakShift) 
      , noise_(noise)
      , strength_dBm_(20.*std::log10(  std::pow(10., peakRef_->strength()  /20.)
                                     + std::pow(10., peakShift_->strength()/20.)))
      , ratio_dB_(strength_dBm_ - noise_->strength()) {}

    virtual ~FFTResultFSKStrength() {}
    virtual std::string toString() const { 
      std::stringstream ss; 
      ss << Base::toString() 
         << " fReference="     << peakRef_->fReference()
         << " fShift="         << peakShift_->fReference()
         << " fMeasuredRef="   << peakRef_->fMeasured()
         << " fMeasuredShift=" << peakShift_->fMeasured()
         << " strength="       << strength_dBm_
         << " strengthRef="    << peakRef_->strength()
         << " strengthShift="  << peakShift_->strength()
         << " noise="          << noise_->strength()
         << " ratio="          << ratio_dB_;
      return ss.str();
    }
    virtual boost::filesystem::fstream& dumpHeader(boost::filesystem::fstream& os) const {      
      Base::dumpHeader(os) 
        << "fReference_Hz fShift_Hz fMeasuredRef_Hz fMeasuredShift_Hz "
        << "strength_dBm strengthRef_dBm strengthShift_dBm S/N_dB ";
      return os;
    }
    virtual boost::filesystem::fstream& dumpData(boost::filesystem::fstream& os) const {
      Base::dumpData(os)
        << boost::format("%12.3f") % peakRef_->fReference() << " "
        << boost::format("%12.3f") % peakShift_->fReference() << " "
        << boost::format("%12.3f") % peakRef_->fMeasured() << " "
        << boost::format("%12.3f") % peakShift_->fMeasured() << " "
        << boost::format("%7.2f")  % strength_dBm_ << " "
        << boost::format("%7.2f")  % peakRef_->strength() << " "
        << boost::format("%7.2f")  % peakShift_->strength() << " "
        << boost::format("%7.2f")  % ratio_dB_ << " ";
      return os;
    }

  private:
    const Result::SpectrumPeak::Handle peakRef_;
    const Result::SpectrumPeak::Handle peakShift_;
    const Result::SpectrumPowerInInterval::Handle noise_;
    const double strength_dBm_;
    const double ratio_dB_;
  } ;
} // namespace Result

#endif // _FFT_RESULT_FSK_PEAK_STRENGTH_cm101118_
