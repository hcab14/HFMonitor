// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_RESULT_SPECTRUM_POWER_IN_INTERVAL_HPP_cm101026_
#define _FFT_RESULT_SPECTRUM_POWER_IN_INTERVAL_HPP_cm101026_

#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>

#include "FFTResult.hpp"
#include "FFTResultCalibration.hpp"
#include "Spectrum.hpp"

namespace Result {
  class SpectrumPowerInInterval : public Base {
  public:
    typedef boost::shared_ptr<SpectrumPowerInInterval> Handle;
    typedef frequency_vector<double> PowerSpectrum;

    SpectrumPowerInInterval(double fReference, 
                            double bandwidth,
                            double normWindow)
      : Base("SpectrumPowerInInterval")
      , fReference_(fReference) 
      , bandwidth_(bandwidth)
      , normWindow_(normWindow)
      , strength_(0.) 
      , averageStrength_(0.) 
      , strengthRMS_(1.) {}

    virtual ~SpectrumPowerInInterval() {}

    bool proc(const PowerSpectrum& ps) { 
      try {
        const std::pair<double, double> fMin(cal(fReference() - 0.5*bandwidth()));
        const std::pair<double, double> fMax(cal(fReference() + 0.5*bandwidth()));
        const size_t indexBeg(ps.freq2index(fMin.first));
        const size_t indexEnd(ps.freq2index(fMax.first));

        double sum(0), sum2(0);
        size_t counter(0);
        for (size_t u(indexBeg); u<=indexEnd; ++u, ++counter) {
          sum += ps[u].second;
          sum2 += ps[u].second * ps[u].second;
        }
        strength_        = sum / normWindow(); // here we do _not_ correct for window gain
        averageStrength_ = (counter != 0) ? sum/counter : 0.;
        strengthRMS_     = (counter != 0) ? std::sqrt(sum2/counter-averageStrength_*averageStrength_) : 1.;
        return true;
      } catch (const std::runtime_error& e) {
        std::cout << "SpectrumPowerInInterval::proc " << e.what() << std::endl;
        return false;
      }
    }

    virtual std::string toString() const { 
      std::stringstream ss; 
      ss << Base::toString() 
         << " fReference="      << fReference()
         << " strength="        << strength()
         << " averageStrength=" << averageStrength()
         << " strengthRMS="     << strengthRMS()
         << " bandwidth="       << bandwidth();
      return ss.str();
    }
    double fReference() const { return fReference_; }
    double bandwidth() const { return bandwidth_; }
    double normWindow() const { return normWindow_; }
    double strength() const { return strength_; }
    double averageStrength() const { return averageStrength_; }
    double strengthRMS() const { return strengthRMS_; }

    // this method is overwritten e.g. in CalibratedSpectrumPeak
    virtual std::pair<double, double> cal(double f) const {
      return std::make_pair(f, double(1));
    }

    virtual boost::filesystem::fstream& dumpHeader(boost::filesystem::fstream& os,
                                                   boost::posix_time::ptime t) const {      
      Base::dumpHeader(os, t) 
        << "fReference_Hz Bandwidth_Hz strength_dBm averageStrength_dBm strengthRMS_dBm ";
      return os;
    }
    virtual boost::filesystem::fstream& dumpData(boost::filesystem::fstream& os,
                                                 boost::posix_time::ptime t) const {
      Base::dumpData(os, t)
        << boost::format("%12.3f") % fReference() << " "
        << boost::format("%9.3f")  % bandwidth() << " "
        << boost::format("%7.2f")  % (20.*std::log10(strength())) << " "
        << boost::format("%7.2f")  % (20.*std::log10(averageStrength())) << " "
        << boost::format("%7.2f")  % (20.*std::log10(strengthRMS())) << " ";
      return os;
    }

  private:
    double fReference_;
    double bandwidth_;
    double normWindow_;
    double strength_;
    double averageStrength_;
    double strengthRMS_;
  } ;

  class CalibratedSpectrumPowerInInterval : public SpectrumPowerInInterval {
  public:
    typedef boost::shared_ptr<SpectrumPeak> Handle;
    CalibratedSpectrumPowerInInterval(double fReference,
                                      double bandwidth,
                                      double normWindow,
                                      Calibration::Handle calibrationHandle)
      : SpectrumPowerInInterval(fReference, bandwidth, normWindow)
      , calibrationHandle_(calibrationHandle) {
      name_= "CalibratedSpectrumPowerInInterval";
    }
    virtual ~CalibratedSpectrumPowerInInterval() {}

    virtual std::pair<double, double> cal(double f) const {
      return calibrationHandle_->uncal2cal(f);
    }

  private:
    Calibration::Handle calibrationHandle_;
  } ;
} // namespace Result

#endif // _FFT_RESULT_SPECTRUM_POWER_IN_INTERVAL_HPP_cm101026_
