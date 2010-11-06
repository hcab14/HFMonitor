// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_ACTION_SPECTRUM_INTERVAL_HPP_cm101026_
#define _FFT_ACTION_SPECTRUM_INTERVAL_HPP_cm101026_

#include <iostream>
#include <string>
#include <stdexcept>
#include <boost/noncopyable.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "Spectrum.hpp"
#include "FFTResult.hpp"
#include "FFTProxy.hpp"

namespace Action {
  class SpectrumInterval : public Base {
  public:
    typedef frequency_vector<double> PowerSpectrum;
    SpectrumInterval(const boost::property_tree::ptree& config)
      : Base("SpectrumInterval")
      , ps_(config.get<double>("fMin"), 
            config.get<double>("fMax"))
      , filterType_(config.find("Filter") != config.not_found() 
                    ? config.get<std::string>("Filter.Type")
                    : "None")
      , filterTimeConstant_((filterType_ != "None") 
                            ? config.get<double>("Filter.TimeConstant")
                            : 1.0)
      , resultKey_(config.get<std::string>("Name"))
      , useCalibration_(config.find("Calibration") != config.not_found())
      , calibrationKey_(useCalibration_ ? config.get<std::string>("Calibration") : "")
      , plotSpectrum_(config.get<bool>("PlotSpectrum", false)) {}

    virtual ~SpectrumInterval() {}    

    std::string resultKey() const { return resultKey_; }
    bool useCalibration() const { return useCalibration_; }
    std::string calibrationKey() const { return calibrationKey_; }
    bool plotSpectrum() const { return plotSpectrum_; }

    virtual void perform(Proxy::Base& p, const SpectrumBase& s) {
      try {
        if (filterType_ == "None") {
          ps_.fill(s, std::abs<double>);
        } else if (filterType_ == "LowPass") {
          if (ps_.empty()) {
            ps_.fill(s, std::abs<double>);
          } else {
            const double dt(s.size()/s.sampleRate());
            const double x(dt / filterTimeConstant_);
            ps_ *= (1-x);
            ps_ += x * PowerSpectrum(ps_.fmin(), ps_.fmax(), s, std::abs<double>);
          }
        } else {
          throw std::runtime_error(filterType_ + " unknown filter");
        }
        // call virtual method
        proc(p, s, ps_);
      } catch (const std::runtime_error& e) {
        std::cout << "SpectrumInterval::perform " << e.what() << std::endl;
      } catch (...) {
        std::cout << "SpectrumInterval::perform unknown error" << std::endl;
      }
    }

    // this method is overwritten by, e.g., FindPeak, see below
    virtual void proc(Proxy::Base& p, 
                      const SpectrumBase& s,
                      const PowerSpectrum& ps) = 0;
    
  protected:
  private:
    PowerSpectrum ps_;
    const std::string filterType_;
    const double filterTimeConstant_;
    const std::string resultKey_;
    const bool useCalibration_;
    const std::string calibrationKey_;
    const bool plotSpectrum_;
  } ;
} // namespace Action

#endif // _FFT_ACTION_SPECTRUM_INTERVAL_HPP_cm101026_
