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
#include "FFTResultSpectrumPeak.hpp"
#include "FFTResultCalibration.hpp"
#include "FFTResultCalibratedSpectrumPeak.hpp"
#include "FFTResultSpectrumPowerInInterval.hpp"
#include "FFTResultFSKStrength.hpp"
#include "FFTResultPowerSpectrumLine.hpp"
#include "FFTProxy.hpp"
#include "FFTActionSpectrumInterval.hpp"

namespace Action {
  class FSKStrength : public SpectrumInterval {
  public:
    FSKStrength(const boost::property_tree::ptree& config)
      : SpectrumInterval(addKeysToConfig(config,
                                         config.get<double>("fRef"),
                                         config.get<double>("fShift")))
      , fRef_(config.get<double>("fRef"))
      , fShift_(config.get<double>("fShift"))
      , minRatio_(config.get<double>("minRatio")) {
      name_ = "FSKStrength";
    }

    virtual ~FSKStrength() {}
    virtual void proc(Proxy::Base& p, 
                      const SpectrumBase& s,
                      const PowerSpectrum& ps) {
      std::cout << "FSKStrength::perform " << std::endl;
      try {
        Result::SpectrumPeak::Handle
          sppRef((useCalibration())
                 ? boost::make_shared<Result::CalibratedSpectrumPeak>
                   (fRef_-fShift_, boost::dynamic_pointer_cast<Result::Calibration>(p.getResult(calibrationKey())))
                 : boost::make_shared<Result::SpectrumPeak>(fRef_-fShift_));

        Result::SpectrumPeak::Handle
          sppShift((useCalibration())
                   ? boost::make_shared<Result::CalibratedSpectrumPeak>
                     (fRef_+fShift_, boost::dynamic_pointer_cast<Result::Calibration>(p.getResult(calibrationKey())))
                   : boost::make_shared<Result::SpectrumPeak>(fRef_+fShift_));

        Result::SpectrumPowerInInterval::Handle
          spiRef((useCalibration())
                 ? boost::make_shared<Result::CalibratedSpectrumPowerInInterval>
                   (fRef_, 1.5*fShift_,
                    boost::dynamic_pointer_cast<Result::Calibration>(p.getResult(calibrationKey())))
                 : boost::make_shared<Result::SpectrumPowerInInterval>(fRef_, 1.5*fShift_));
        
        sppRef->findPeak  (s, ps, fRef_+0.5*fShift_, fRef_+1.5*fShift_, 1.0);
        sppShift->findPeak(s, ps, fRef_-1.5*fShift_, fRef_-0.5*fShift_, 1.0);
        spiRef->proc(ps);

        const double strengthFSK(sppRef->strength() + sppShift->strength());
        const double strengthBackground(spiRef->strength());

        if (strengthFSK/strengthBackground > minRatio_)
          p.putResult(resultKey(),
                      Result::FFTResultFSKStrength::Handle
                      (new Result::FFTResultFSKStrength(sppRef, sppShift, spiRef)));        
        if (plotSpectrum())
          p.putResult(resultKey()+"_plot",
                      boost::make_shared<Result::PowerSpectrumLine>(p.getApproxPTime(), ps));
      } catch (const std::runtime_error& e) {
        std::cout << e.what() << std::endl;
      }
    }
  private:
    // add keys "fMin" and "fMax"
    static boost::property_tree::ptree addKeysToConfig(const boost::property_tree::ptree& config,
                                                       double fRef,
                                                       double fShift) {
      boost::property_tree::ptree result(config);
      result.put("fMin", boost::lexical_cast<std::string>(fRef-2*fShift));
      result.put("fMax", boost::lexical_cast<std::string>(fRef+2*fShift));
      return result;
    }
    const double fRef_;            // nominal (mark) frequency / Hz
    const double fShift_;          // shift frequency / Hz
    const double minRatio_;        // min. ratio peak/background
  } ;
} // namespace Action

#endif // _FFT_ACTION_FSK_STRENGTH_HPP_cm101118_
