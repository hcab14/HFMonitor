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
#include "Filter.hpp"
#include "FFTResult.hpp"
#include "FFTProxy.hpp"

namespace Action {
  class SpectrumInterval : public Base {
  public:
    typedef frequency_vector<double> PowerSpectrum;
    typedef Filter::Cascaded<PowerSpectrum> filter_type;
    SpectrumInterval(const boost::property_tree::ptree& config)
      : Base("SpectrumInterval")
      , fMin_(config.get<double>("fMin"))
      , fMax_(config.get<double>("fMax"))
      , resultKey_(config.get<std::string>("Name"))
      , useCalibration_(config.find("Calibration") != config.not_found())
      , calibrationKey_(useCalibration_ ? config.get<std::string>("Calibration") : "")
      , plotSpectrum_(config.get<bool>("PlotSpectrum", false)) {
      if (config.find("Filter") != config.not_found()) {
        if (config.get<std::string>("Filter.Type") == "None") {
          // nop
        } else if (config.get<std::string>("Filter.Type") == "LowPass") {
          filter_.add(Filter::LowPass<PowerSpectrum>::make(1.0, config.get<double>("Filter.TimeConstant")));
        } else {
          throw std::runtime_error(config.get<std::string>("Filter.Type") + ": unknown filter");
        }
      }
    }

    virtual ~SpectrumInterval() {}    

    std::string resultKey() const { return resultKey_; }
    bool useCalibration() const { return useCalibration_; }
    std::string calibrationKey() const { return calibrationKey_; }
    bool plotSpectrum() const { return plotSpectrum_; }

    virtual void perform(Proxy::Base& p, const SpectrumBase& s) {
      try {
        const PowerSpectrum ps(fMin_, fMax_, s, std::abs<double>);        
        if (filter_.x().empty()) {
          filter_.init(p.getApproxPTime(), ps);
        } else {
          filter_.update(p.getApproxPTime(), ps);
        }
        // call virtual method
        proc(p, s, filter_.x());
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
    filter_type filter_;
    const double fMin_;
    const double fMax_;
    const std::string resultKey_;
    const bool useCalibration_;
    const std::string calibrationKey_;
    const bool plotSpectrum_;
  } ;
} // namespace Action

#endif // _FFT_ACTION_SPECTRUM_INTERVAL_HPP_cm101026_
