// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FILTER_HPP_cm101017_
#define _FILTER_HPP_cm101017_

#include <vector>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Filter {
  template<typename T>
  class Base : private boost::noncopyable {
  public:
    typedef typename boost::shared_ptr<Base<T> > sptr;
    virtual ~Base() {}
    virtual void init(boost::posix_time::ptime t, const T& x) = 0;
    virtual T update(boost::posix_time::ptime t, const T& x) = 0 ;
    virtual T x() const = 0;
    virtual bool isInEquilibrium() const = 0;    
  } ;

  template<typename T>
  class LowPassBase : public Base<T> {
  public:
    typedef typename boost::shared_ptr<LowPassBase<T> > sptr;
    LowPassBase(double dt, double lambda)
      : dt_(dt)
      , lambda_(lambda)
      , a_(dt_/lambda_) {}
    virtual ~LowPassBase() {}

    using Base<T>::update;
    using Base<T>::x;

    virtual void init(boost::posix_time::ptime t, const T& x0) {
      t0_ = t_ = t;
      x_  = x0;
    }
    double dt() const { return dt_; }
    double lambda() const { return lambda_; }

    virtual bool isInEquilibrium() const { 
      using namespace boost::posix_time;
      return (t_ - t0_) > time_duration(0,0,0, 2*lambda_*time_duration::ticks_per_second());
    }
  protected:
    double dt_;
    double lambda_;
    double a_;
    T x_;
    boost::posix_time::ptime t0_; // filter start time
    boost::posix_time::ptime t_;  // current filter time
  } ;

  template<typename T>
  class LowPass : public LowPassBase<T> {
  public:
    typedef boost::shared_ptr<LowPass<T> > sptr;
    LowPass(double dt, double lambda)
      : LowPassBase<T>(dt, lambda) {}
    virtual ~LowPass() {}

    static typename Base<T>::sptr make(double dt, double lambda) {
      return typename Base<T>::sptr(new LowPass(dt, lambda));
    }
    virtual T x() const { return x_; }
    virtual T update(boost::posix_time::ptime pt, const T& xx) {
      using namespace boost::posix_time;
      const double dt(double((pt-t_).ticks()) / double(time_duration::ticks_per_second()));
      t_ = pt;
      a_ = dt / lambda_;
      x_ *= (1-a_); x_ += a_*xx;
      return x();
    }
  private:
    using LowPassBase<T>::x_;
    using LowPassBase<T>::a_;
    using LowPassBase<T>::t_;
    using LowPassBase<T>::lambda_;
  } ;

  class PTimeLowPass : public LowPassBase<boost::posix_time::ptime> {
  public:
    typedef boost::shared_ptr<PTimeLowPass> sptr;
    PTimeLowPass(double dt, double lambda)
      : LowPassBase<boost::posix_time::ptime>(dt, lambda)
      , secondsInDay_(0) {}
    virtual ~PTimeLowPass() {}

    static Base<boost::posix_time::ptime>::sptr make(double dt, double lambda) {
      return Base<boost::posix_time::ptime>::sptr(new PTimeLowPass(dt, lambda));
    }

    virtual void init(boost::posix_time::ptime t0,
                      const boost::posix_time::ptime& t) {
      using namespace boost::posix_time;
      // compensation for the filter delay
      const ptime tCorr(t - time_duration(0,0,0, (lambda_-dt_)*time_duration::ticks_per_second()));
      LowPassBase<ptime>::init(t0,tCorr);
      secondsInDay_ = 1e-6*tCorr.time_of_day().total_microseconds();
    }        
    virtual boost::posix_time::ptime update(boost::posix_time::ptime , 
                                            const boost::posix_time::ptime& pt) {
      return this->update(pt);
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
      // 1. compute the filter time corrected for day rollover _relative to ptime_
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
      if (dt >  12*3600) return  24*3600;
      if (dt < -12*3600) return -24*3600;
      return 0;
    }    
    double secondsInDay_;  // filter state 
  } ;

  template<typename T>
  class WithRMS : Base<T> {
  public:
    WithRMS(typename Base<T>::sptr filterX,
            typename Base<T>::sptr filterRMS,
            const T& initRMS)
      : filterX_(filterX)
      , filterRMS_(filterRMS)
      , initRMS_(initRMS) {}
    virtual ~WithRMS() {}

    virtual void init(boost::posix_time::ptime pt, const T& x) {
      filterX_->init(pt, x);
      filterRMS_->init(pt, initRMS_*initRMS_);
    }
    virtual T update(boost::posix_time::ptime pt, const T& x) {
      const T res(filterX_->update(pt, x.first));
      filterRMS_->update(pt, (x-res)*(x-res));
      return res;
    }
    virtual T x() const   { return filterX_.x(); }
    virtual T rms() const { return sqrt(filterRMS_.x()); }
    virtual bool isInEquilibrium() const {
      if (filterX_ == 0 || filterRMS_ == 0)
        return false;
      return filterX_->isInEquilibrium() && filterRMS_->isInEquilibrium();
    }

  protected:
  private:
    typename Base<T>::sptr filterX_;
    typename Base<T>::sptr filterRMS_;
    const T initRMS_;
  } ;

  template<typename T>
  class Cascaded : Base<T> {
  public:
    Cascaded() {}
    virtual ~Cascaded() {}

    void add(typename Base<T>::sptr fp) {
      filters_.push_back(fp);
    }
    virtual void init(boost::posix_time::ptime pt, const T& x) {
      BOOST_FOREACH(typename Base<T>::sptr filter, filters_)
        filter->init(pt, x);
    }
    virtual T update(boost::posix_time::ptime pt, const T& x) {
      T result(x);
      BOOST_FOREACH(typename Base<T>::sptr filter, filters_)
        result = filter->update(pt, result);
      return result;
    }
    virtual T x() const {
      return filters_.back()->x();
    }
    virtual bool isInEquilibrium() const {
      if (filters_.empty())
        return false;
      BOOST_FOREACH(typename Base<T>::sptr filter, filters_)
        if (not filter->isInEquilibrium()) return false;
      return true;
    }
  private:
    std::vector<typename Base<T>::sptr> filters_;
  } ;
}
#endif // _FILTER_HPP_cm101017_
