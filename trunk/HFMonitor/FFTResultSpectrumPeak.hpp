// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_RESULT_SPECTRUM_PEAK_HPP_cm101026_
#define _FFT_RESULT_SPECTRUM_PEAK_HPP_cm101026_

#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>

#include "FFTResultCalibration.hpp"
#include "Spectrum.hpp"

namespace Result {
  class SpectrumPeak : public Base {
  public:
    typedef boost::shared_ptr<SpectrumPeak> Handle;
    typedef frequency_vector<double> PowerSpectrum;
    SpectrumPeak(double fReference)
      : Base("SpectrumPeak")
      , fReference_(fReference) 
      , fMeasured_(0.) 
      , fMeasuredRMS_(1.) 
      , strength_(0.) 
      , strengthRMS_(1.)
      , ratio_(1.) {}

    bool findPeak(const SpectrumBase& s, const PowerSpectrum& ps, double minRatio) {
      PowerSpectrum::const_iterator
        iMin(std::min_element(ps.begin(), ps.end(), PowerSpectrum::cmpSecond));
      PowerSpectrum::const_iterator
        iMax(std::max_element(ps.begin(), ps.end(), PowerSpectrum::cmpSecond));
      
      const size_t indexMax(std::distance(ps.begin(), iMax));

      double sum(0.0);
      double weight(0.0);
      for (size_t u(indexMax-std::min(indexMax, size_t(3))); u<(indexMax+4) && u < ps.size(); ++u) {
        weight += ps[u].second;
        sum += ps[u].second * ps[u].first;
      }
      for (int i=-2; i<=2; ++i) {
        std::cout << "FP_ " << ps[indexMax+i].first << " " << ps[indexMax+i].second << " "
                  << std::arg(s[s.freq2index(ps[indexMax+i].first)])/(2*M_PI) 
		  << std::endl;
      }
      ratio_ = (iMin->second != 0.0) ? iMax->second / iMin->second : 1.0;
      if (ratio_ < minRatio || weight == 0.0) {
        std::cout << "ratio < minRatio || weight == 0.0: " << ratio_ << " " << sum << " " << weight << std::endl;
        return false;
      }
      // TODO: error propagation
      const std::pair<double, double> c(cal(sum/weight));
      fMeasured_    = c.first;
      fMeasuredRMS_ = c.second;
      strength_     = iMax->second;
      strengthRMS_  = iMax->second/10.; // TODO
      return true;
    }

    virtual ~SpectrumPeak() {}
    virtual std::string toString() const { 
      std::stringstream ss; 
      ss << Base::toString() 
         << " fReference="   << fReference()
         << " fMeasured="    << fMeasured()
         << " fMeasuredRMS=" << fMeasuredRMS()
         << " strength="     << strength()
         << " strengthRMS="  << strengthRMS()
         << " ratio="        << ratio();
      return ss.str();
    }
    double fReference() const { return fReference_; }
    double fMeasured() const { return fMeasured_; }
    double fMeasuredRMS() const { return fMeasuredRMS_; }
    double strength() const { return strength_; }
    double strengthRMS() const { return strengthRMS_; }
    double ratio() const { return ratio_; }

    // this method is overwritten e.g. in CalibratedSpectrumPeak
    virtual std::pair<double, double> cal(double f) const {
      return std::make_pair(f, double(1));
    }

    virtual boost::filesystem::fstream& dumpHeader(boost::filesystem::fstream& os,
                                                   boost::posix_time::ptime t) const {      
      Base::dumpHeader(os, t) 
        << "fReference_Hz fMeasured_Hz fMeasuredRMS_Hz strength_dBm strengthRMS_dBm S/N_dB ";
      return os;
    }
    virtual boost::filesystem::fstream& dumpData(boost::filesystem::fstream& os,
                                                 boost::posix_time::ptime t) const {
      Base::dumpData(os, t)
        << boost::format("%12.3f") % fReference() << " "
        << boost::format("%12.3f") % fMeasured() << " "
        << boost::format("%6.3f")  % fMeasuredRMS() << " "
        << boost::format("%7.2f")  % (20.*std::log10(strength())) << " "
        << boost::format("%7.2f")  % (20.*std::log10(strengthRMS())) << " "
        << boost::format("%7.2f")  % (20.*std::log10(ratio())) << " ";
      return os;
    }

  private:
    double fReference_;
    double fMeasured_;
    double fMeasuredRMS_;
    double strength_;
    double strengthRMS_;
    double ratio_;
  } ;
} // namespace Result

#endif // _FFT_RESULT_SPECTRUM_PEAK_HPP_cm101026_
