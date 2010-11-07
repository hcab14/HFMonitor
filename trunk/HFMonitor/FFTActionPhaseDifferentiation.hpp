// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_ACTION_PHASE_DIFFERENTIATION_HPP_cm101106_
#define _FFT_ACTION_PHASE_DIFFERENTIATION_HPP_cm101106_

#include <iostream>
#include <string>
#include <stdexcept>
#include <boost/noncopyable.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "Spectrum.hpp"
#include "Filter.hpp"
#include "FFTResult.hpp"
#include "FFTProxy.hpp"

namespace Action {
  class PhaseDifferentiation : public Base {
  public:
    typedef Filter::WithRMS<double, frequency_vector> filter_type;
    typedef filter_type::vector_type PhaseSpectrum;
    typedef boost::posix_time::ptime ptime;

    PhaseDifferentiation(const boost::property_tree::ptree& config)
      : Base("SpectrumInterval")
      , ps_(config.get<double>("fMin"), 
            config.get<double>("fMax"))
      , filterDiffPhase_(Filter::LowPass<PhaseSpectrum>::make(1.0, config.get<double>("Filter.TimeConstant")),
                         Filter::LowPass<PhaseSpectrum>::make(1.0, config.get<double>("Filter.TimeConstant")), 
                         1.0)
      , resultKey_(config.get<std::string>("Name"))
      , plotSpectrum_(config.get<bool>("PlotSpectrum", false)) {}

    virtual ~PhaseDifferentiation() {}    

    std::string resultKey() const { return resultKey_; }
    bool plotSpectrum() const { return plotSpectrum_; }
    static double mod1(double x) {
      return ((x>1.) ? x-1. :
              (x<0.) ? x+1. : x);
    }
    virtual void perform(Proxy::Base& p, const SpectrumBase& s) {
      using namespace boost::posix_time;
      try {
        const double dt(  double((p.getApproxPTime() - t_).ticks())
                        / double(time_duration::ticks_per_second()));
        if (ps_.empty()) {
          ps_.fill(s, std::arg<double>);
          t_= p.getApproxPTime();
        } else {
          PhaseSpectrum psNew(ps_.fmin(), ps_.fmax(), s, std::arg<double>);          
          PhaseSpectrum diffPhaseNew((psNew-ps_) / (2*M_PI)); // 2*pi = 1
          diffPhaseNew.apply(mod1);
          if (filterDiffPhase_.x().empty()) {
            filterDiffPhase_.init(p.getApproxPTime(), diffPhaseNew);
          } else {
            filterDiffPhase_.update(p.getApproxPTime(), diffPhaseNew);
          }
          ps_ = psNew;
          t_= p.getApproxPTime();
        }
        // call virtual method
        proc(p, s, 
             filterDiffPhase_.x(), 
             filterDiffPhase_.rms(), 
             dt);
      } catch (const std::runtime_error& e) {
        std::cout << "PhaseDifferentiation::perform " << e.what() << std::endl;
      } catch (...) {
        std::cout << "PhaseDifferentiation::perform unknown error" << std::endl;
      }
    }

    // this method can be overwritten
    virtual void proc(Proxy::Base& p, 
                      const SpectrumBase& s,
                      const PhaseSpectrum& ps,
                      const PhaseSpectrum& psrms,
                      double dt) {
      std::cout << "procproc " << std::endl;
      for (unsigned u=0; u<ps.size(); ++u)
        std::cout << "  --- " << dt << " " << ps[u].first << " " << ps[u].second << " " << psrms[u].second << std::endl;
    }
    
  protected:
  private:
    PhaseSpectrum ps_;        //
    ptime         t_;         // time when ps_ was taken
    filter_type filterDiffPhase_;
    const std::string filterType_;
    const std::string resultKey_;
    const bool plotSpectrum_;
  } ;
} // namespace Action

#endif // _FFT_ACTION_PHASE_DIFFERENTIATION_HPP_cm101106_
