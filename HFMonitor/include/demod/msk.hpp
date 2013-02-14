// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _DEMOD_MSK_HPP_cm130214_
#define _DEMOD_MSK_HPP_cm130214_

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

#include "filter/iir.hpp"
#include "filter/pll.hpp"

namespace demod {  
  class msk : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<msk> sptr;

    typedef filter::loop_filter_2nd<double> loop_filter;
    typedef filter::integrator_modulo<double> integrator;
    typedef filter::pll<double, loop_filter, integrator> pll_type;
    typedef pll_type::complex_type complex_type;

    static sptr make(double fs_Hz,  // sampling frequency
                     double fc_Hz,  // frequency of modulation
                     double dwl,    // "bandwidth" of PLL
                     double xi  = 1./sqrt(2)) {
      return sptr(new msk(fs_Hz, fc_Hz, dwl, xi));
    }

    void process(complex_type s) {
      const complex_type s2(s*s);
      pll_plus_.process(s2);
      pll_minus_.process(s2);      
    }

    const pll_type& pll_plus()  const { return pll_plus_; }
    const pll_type& pll_minus() const { return pll_minus_; }

  protected:
  private: 
    static pll_type make_pll(double fs_Hz,
                             double fc_Hz,
                             double dwl,
                             double xi) {
      loop_filter l(xi, dwl, fc_Hz, fs_Hz);
      integrator  i(4*M_PI);
      return pll_type(fc_Hz, fs_Hz, l, i);
    }

    msk(double fs_Hz,
        double fc_Hz,
        double dwl,
        double xi)
      : pll_plus_ (make_pll(fs_Hz,  fc_Hz, dwl, xi))
      , pll_minus_(make_pll(fs_Hz, -fc_Hz, dwl, xi)) {}

    pll_type    pll_plus_;
    pll_type    pll_minus_;
  } ;

} // namespace demod
#endif //_DEMOD_MSK_HPP_cm130214_
