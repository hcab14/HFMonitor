// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_ACTION_HPP_cm101026_
#define _FFT_ACTION_HPP_cm101026_

#include <iostream>
#include <string>
#include <boost/noncopyable.hpp>

#include "Spectrum.hpp"
#include "FFTResult.hpp"
#include "FFTProxy.hpp"

namespace Action {
  class Base : private boost::noncopyable {
  public:
    Base(std::string name)
      : name_(name) {}
    virtual ~Base() {}
    std::string name() const { return name_; }
    virtual void perform(Proxy::Base& p, const SpectrumBase& s) = 0;
  protected:
    std::string name_;
  } ;

  class SpectumInterval : public Base {
  public:
    SpectumInterval(const boost::property_tree::ptree& config)
      : Base("SpectumInterval")
      , ps_(config.get<double>("fMin"), 
            config.get<double>("fMax"))
      , filterType_(config.find("Filter") != config.not_found() 
                    ? config.get<std::string>("Filter.Type")
                    : "None")
      , filterTimeConstant_((filterType_ != "None") 
                            ? config.get<double>("Filter.TimeConstant")
                            : 1.0) {}
    
    virtual void perform(Proxy::Base& p, const SpectrumBase& s) {
      try {
        if (filterType_ == "None") {
          ps_.clear();
          ps_.fill(s);
        } else if (filterType_ == "LowPass") {
          if (ps_.empty()) {
            ps_.fill(s);
        } else {
            const double dt(s.size()/s.sampleRate());
            const double x(dt / filterTimeConstant_);
            ps_ *= (1-x);
            ps_ += x * PowerSpectrum(ps_.fMin(), ps_.fMax(), s);
        }
      } else {
        throw std::runtime_error(filterType_ + " unknown filter");
      }
      // call virtual method
      proc(p, s, ps_);      
      } catch (...) {
        return;
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
  } ;

  class FindPeak : public SpectumInterval {
  public:
    FindPeak(const boost::property_tree::ptree& config)
      : SpectumInterval(config)
      , fReference_(config.get<double>("fRef"))
      , minRatio_(config.get<double>("minRatio"))
      , resultKey_(config.get<std::string>("Name"))
      , useCalibration_(config.find("Calibration") != config.not_found())
      , calibrationKey_(useCalibration_ ? config.get<std::string>("Calibration") : "")
      , plotSpectrum_(config.get<bool>("PlotSpectrum", false)) {
      name_ = "FindPeak";
    }
    
    virtual void proc(Proxy::Base& p, 
                      const SpectrumBase& s,
                      const PowerSpectrum& ps) {
      std::cout << "FindPeak::perform " << std::endl;
      try {
        Result::SpectrumPeak::Handle 
          spp((useCalibration_)
              ? boost::make_shared<Result::CalibratedSpectrumPeak>
              (fReference_, boost::dynamic_pointer_cast<Result::Calibration>(p.getResult(calibrationKey_)))
              : boost::make_shared<Result::SpectrumPeak>(fReference_));
        if (spp->findPeak(ps, minRatio_))
          p.putResult(resultKey_, spp);
        if (plotSpectrum_)
          p.putResult(resultKey_+"_plot",
                      boost::make_shared<Result::PowerSpectrumLine>(ps.pgmLine(p.getApproxPTime())));
      } catch (const std::runtime_error& e) {
        std::cout << e.what() << std::endl;
      }
    }
  private:
    const double fReference_;          // nominal frequency / Hz
    const double minRatio_;            // min. ratio peak/background
    const std::string resultKey_;      // result key name
    const bool useCalibration_;        // 
    const std::string calibrationKey_; // key name for calibration information
    const bool plotSpectrum_;          //
  } ;

  class Calibrator : public Base {
  public:
    Calibrator(const boost::property_tree::ptree& config)
      : Base("Calibrator")
      , resultKey_(config.get<std::string>("Name")) {
      using boost::property_tree::ptree;
      const ptree& pt(config.get_child("Inputs"));
      for (ptree::const_iterator i(pt.begin()); i!=pt.end(); ++i) {
        if (i->first == "Input") {
          std::cout << "Calibrator::Calibrator Input." << i->second.get<std::string>("") << std::endl;
          inputs_.push_back(i->second.get<std::string>(""));
        } else {
          std::cout << "Calibrator::calibrate unknown field " << i->first << std::endl;
        }
      }
    }

    virtual void perform(Proxy::Base& p, const SpectrumBase& s) {
      std::cout << "Calibrator::perform" << std::endl;
      // count data
      std::vector<Result::SpectrumPeak::Handle> peaks;
      for (std::vector<std::string>::const_iterator i(inputs_.begin()); i!=inputs_.end(); ++i) {
        try {
          Result::SpectrumPeak::Handle sp(boost::dynamic_pointer_cast<Result::SpectrumPeak>(p.getResult(*i)));
          if (sp != 0) {
            peaks.push_back(sp);
          }
        } catch (...) {
          // TODO ...
        }
      }
      try {     
        Result::Base::Handle rh(new Result::Calibration(peaks));
        p.putResult(resultKey_, rh); 
      } catch (...) {
        // ...
      }
    }
  private:
    std::string resultKey_;           // result key name
    std::vector<std::string> inputs_; // key names of inputs used for calibration
  } ;

  typedef boost::shared_ptr<Base> Handle;

  struct Factory {
    static Handle makeAction(std::string name, const boost::property_tree::ptree& pt) {
      if (name == "FindPeak")
        return Handle(new FindPeak(pt));
      if (name == "Calibrator")
        return Handle(new Calibrator(pt));
      else
        throw std::runtime_error(name+": action not supported");
    }
  } ;
} // namespace Action

#endif // _FFT_ACTION_HPP_cm101026_
