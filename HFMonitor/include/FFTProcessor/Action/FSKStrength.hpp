// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_ACTION_FSK_STRENGTH_HPP_cm101118_
#define _FFT_ACTION_FSK_STRENGTH_HPP_cm101118_

#include <iostream>
#include <string>
#include <stdexcept>
#include <boost/noncopyable.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "Spectrum.hpp"
#include "FFTProcessor/Result/SpectrumPeak.hpp"
#include "FFTProcessor/Result/Calibration.hpp"
#include "FFTProcessor/Result/CalibratedSpectrumPeak.hpp"
#include "FFTProcessor/Result/SpectrumPowerInInterval.hpp"
#include "FFTProcessor/Result/FSKStrength.hpp"
#include "FFTProcessor/Result/PowerSpectrumLine.hpp"
#include "FFTProcessor/Proxy.hpp"
#include "FFTProcessor/Action/SpectrumInterval.hpp"

namespace Action {
  class FSKStrength : public SpectrumInterval {
  public:
    FSKStrength(const boost::property_tree::ptree& config)
      : SpectrumInterval(addKeysToConfig(config,
                                         config.get<double>("<xmlattr>.fRef_Hz"),
                                         config.get<double>("<xmlattr>.fShift_Hz")))
      , fRef_(config.get<double>("<xmlattr>.fRef_Hz"))
      , fShift_(config.get<double>("<xmlattr>.fShift_Hz"))
      , minRatio_dB_(config.get<double>("<xmlattr>.minRatio_db")) {
      name_ = "FSKStrength";
    }

    virtual ~FSKStrength() {}
    virtual void proc(Proxy::Base& p, 
                      const SpectrumBase& s,
                      const PowerSpectrum& ps) {
      try {
        const Proxy::Base::ptime t(p.getApproxPTime());
        Result::SpectrumPeak::Handle
          sppRef((useCalibration())
                 ? boost::make_shared<Result::CalibratedSpectrumPeak>
                 (t, fRef_-fShift_, boost::dynamic_pointer_cast<Result::Calibration>(p.getResult(calibrationKey())))
                 : boost::make_shared<Result::SpectrumPeak>(t, fRef_-fShift_));

        Result::SpectrumPeak::Handle
          sppShift((useCalibration())
                   ? boost::make_shared<Result::CalibratedSpectrumPeak>
                   (t, fRef_+fShift_, boost::dynamic_pointer_cast<Result::Calibration>(p.getResult(calibrationKey())))
                   : boost::make_shared<Result::SpectrumPeak>(t, fRef_+fShift_));

        Result::SpectrumPowerInInterval::Handle
          spiRef((useCalibration())
                 ? boost::make_shared<Result::CalibratedSpectrumPowerInInterval>
                 (t, fRef_+1.5*fShift_, 0.25*fShift_, s.normWindow(),
                    boost::dynamic_pointer_cast<Result::Calibration>(p.getResult(calibrationKey())))
                 : boost::make_shared<Result::SpectrumPowerInInterval>(t, fRef_+1.5*fShift_, 0.25*fShift_, s.normWindow()));
        
        sppRef->findPeak  (p, s, ps, fRef_+0.5*fShift_, fRef_+1.5*fShift_, 1.0);
        sppShift->findPeak(p, s, ps, fRef_-1.5*fShift_, fRef_-0.5*fShift_, 1.0);
        spiRef->proc(p, ps);

        const double strengthFSK(sppRef->strength() + sppShift->strength());
        const double strengthBackground(spiRef->strength());

        if (strengthFSK - strengthBackground > minRatio_dB_)
          p.putResult(resultKey(),
                      Result::FFTResultFSKStrength::Handle
                      (new Result::FFTResultFSKStrength(t, sppRef, sppShift, spiRef)));        
        if (plotSpectrum())
          p.putResult(resultKey()+"_plot",
                      boost::make_shared<Result::PowerSpectrumLine>(t, ps));
      } catch (const std::runtime_error& e) {
        LOG_WARNING(e.what());
      }
    }
  private:
    // add keys "<xmlattr>.f{Min,Max}_Hz"
    static boost::property_tree::ptree addKeysToConfig(const boost::property_tree::ptree& config,
                                                       double fRef,
                                                       double fShift) {
      boost::property_tree::ptree result(config);
      result.put("<xmlattr>.fMin_Hz", boost::lexical_cast<std::string>(fRef-2*fShift));
      result.put("<xmlattr>.fMax_Hz", boost::lexical_cast<std::string>(fRef+2*fShift));
      return result;
    }
    const double fRef_;            // nominal (mark) frequency / Hz
    const double fShift_;          // shift frequency / Hz
    const double minRatio_dB_;        // min. ratio peak/background
  } ;
} // namespace Action

#endif // _FFT_ACTION_FSK_STRENGTH_HPP_cm101118_
