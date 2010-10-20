// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FILTER_HPP_cm101017_
#define _FILTER_HPP_cm101017_

#include "boost/date_time/posix_time/posix_time.hpp"

namespace Filter {
  template<typename T>
  class LowPassBase {
  public:
    LowPassBase(double dt, double lambda)
      : dt_(dt)
      , lambda_(lambda)
      , a_(dt_/lambda_) {}
    virtual ~LowPassBase() {}

    void init(boost::posix_time::ptime t, T x0) {
      t0_ = t;
      x_  = x0;
    }
    double dt() const { return dt_; }
    double lambda() const { return lambda_; }
    virtual T x() const { return x_; }

    bool isInEquilibrium() const { 
      using namespace boost::posix_time;
      return (t_ - t0_) > time_duration(0,0,0, 2*lambda_*time_duration::ticks_per_second());
    }

    // virtual T update(boost::posix_time::ptime t, T x) {
    //   t_ = t;
    //   x_ = x_*(1-a_) + x*a_;
    //   return x_;
    // }    

  protected:
    double dt_;
    double lambda_;
    double a_;
    T x_;
    boost::posix_time::ptime t0_; // filter start time
    boost::posix_time::ptime t_;  // current filter time
  } ;

  // typedef LowPassBase<double> ScalarLowPass;

  class PTimeLowPass : public LowPassBase<boost::posix_time::ptime> {
  public:
    typedef LowPassBase<boost::posix_time::ptime> Base;
    PTimeLowPass(double dt, double lambda)
      : Base(dt, lambda)
      , secondsInDay_(0) {}
    virtual ~PTimeLowPass() {}

    void init(boost::posix_time::ptime t) {
      Base::init(t,t);
      secondsInDay_ = 1e-6*t.time_of_day().total_microseconds();
    }        
    boost::posix_time::ptime update(boost::posix_time::ptime pt)  {
      t_ = pt;
      using namespace boost::posix_time;
      // filter input: seconds in day
      const double xNew(double(pt.time_of_day().ticks()) / double(time_duration::ticks_per_second()));
      double xp = (1.-a_) * (secondsInDay_ - rollOverOffset(secondsInDay_-xNew)) + a_ * xNew;
      xp = (xp < 0)        ? xp + 24*3600 : xp;
      xp = (xp >= 24*3600) ? xp - 24*3600 : xp;
      // update filter state
      // 1. conpute the filter time corrected for day rollover _relative to ptime_
      const double tt(xp - rollOverOffset(xp-secondsInDay_));
      x_ = ptime(x_.date()) + time_duration(0,0,0, tt*time_duration::ticks_per_second());
      // 2. update seconds in day
      secondsInDay_ = xp;
      return this->x();
    }
    
    virtual boost::posix_time::ptime x() const { 
      using namespace boost::posix_time;
      // compensate for filter delay
      return x_ + time_duration(0,0,0, (lambda_-dt_)*time_duration::ticks_per_second()); 
    }
  private:  
    static double rollOverOffset(double dt) {
      if (dt >  12*3600) return 24*3600;
      if (dt < -12*3600) return -24*3600;
      return 0;
    }    
    double secondsInDay_;  // filter state 
  } ;
}
#endif // _FILTER_HPP_cm101017_
