// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_ACTION_HPP_cm101026_
#define _FFT_ACTION_HPP_cm101026_

// ___ Implemented in FFTAction

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

  typedef boost::shared_ptr<Base> Handle;

  struct Factory {
    static Handle makeAction(std::string name, const boost::property_tree::ptree& pt);
  } ;
} // namespace Action

#endif // _FFT_ACTION_HPP_cm101026_
