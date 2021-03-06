// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2014 Christoph Mayer
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#ifndef _FILTER_HPP_cm101017_
#define _FILTER_HPP_cm101017_

#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Filter {
  template<typename T>
  class Base : private boost::noncopyable {
  public:
    typedef typename boost::shared_ptr<Base<T> > sptr;
    virtual ~Base() {}
    virtual void init(boost::posix_time::ptime t, const T& x) = 0;
    virtual const T& update(boost::posix_time::ptime t, const T& x) = 0 ;
    virtual const T& x() const = 0;
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
      return (t_ - t0_) > time_duration(0,0,0, boost::int64_t(2*lambda_*time_duration::ticks_per_second()));
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
    virtual const T& x() const { return x_; }
    virtual const T& update(boost::posix_time::ptime pt, const T& xx) {
      using namespace boost::posix_time;
      const double dt(double((pt-t_).ticks()) / double(time_duration::ticks_per_second()));
      t_ = pt;
      a_ = dt / lambda_;
      x_ *= (1-a_); x_ += a_*xx;
      // std::cout << "Filter::Update " << dt << " " << a_<< std::endl;
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
      const ptime tCorr(t - time_duration(0,0,0, boost::int64_t((lambda_-dt_)*time_duration::ticks_per_second())));
      LowPassBase<ptime>::init(t0,tCorr);
      secondsInDay_ = 1e-6*tCorr.time_of_day().total_microseconds();
    }        
    virtual const boost::posix_time::ptime& update(boost::posix_time::ptime , 
                                                   const boost::posix_time::ptime& pt) {
      return this->update(pt);
    }
    const boost::posix_time::ptime& update(boost::posix_time::ptime pt)  {
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
      x_ = ptime(x_.date()) + time_duration(0,0,0, boost::int64_t(tt*time_duration::ticks_per_second()));
      // 2. update seconds in day
      secondsInDay_ = xp;
      return this->x();
    }    
    virtual const boost::posix_time::ptime& x() const { 
      using namespace boost::posix_time;
      // compensate for filter delay
      t_ = x_ + time_duration(0,0,0, boost::int64_t((lambda_-dt_)*time_duration::ticks_per_second()));
      return t_;
    }
  private:  
    static double rollOverOffset(double dt) {
      if (dt >  12*3600) return  24*3600;
      if (dt < -12*3600) return -24*3600;
      return 0;
    }    
    double secondsInDay_;  // filter state 
    mutable boost::posix_time::ptime t_; //
  } ;

  template<typename T,
           template<class > class VECTOR>
  class WithRMS : Base<VECTOR<T> > {
  public:
    typedef  VECTOR<T> vector_type;
    WithRMS(typename Base<vector_type>::sptr filterX,
            typename Base<vector_type>::sptr filterRMS,
            const T& initRMS)
      : filterX_(filterX)
      , filterRMS_(filterRMS)
      , initRMS_(initRMS) {}
    virtual ~WithRMS() {}

    virtual void init(boost::posix_time::ptime pt, const vector_type& x) {
      filterX_->init(pt, x);
      const vector_type rms2(x.fmin(), x.fmax(), x.size(), initRMS_*initRMS_);
      filterRMS_->init(pt, rms2);
    }
    virtual const vector_type& update(boost::posix_time::ptime pt, const vector_type& x) {
      const vector_type& res(filterX_->update(pt, x));
      filterRMS_->update(pt, (x-res)*(x-res));
      return res;
    }
    virtual const vector_type& x() const   { return filterX_->x(); }
    virtual vector_type rms() const { return sqrt(filterRMS_->x()); }
    virtual bool isInEquilibrium() const {
      if (filterX_ == 0 || filterRMS_ == 0)
        return false;
      return filterX_->isInEquilibrium() && filterRMS_->isInEquilibrium();
    }

  protected:
  private:
    typename Base<vector_type>::sptr filterX_;
    typename Base<vector_type>::sptr filterRMS_;
    T initRMS_;
  } ;

  template<typename T>
  class Cascaded : Base<T> {
  public:
    Cascaded() {}
    virtual ~Cascaded() {}

    bool empty() const { return filters_.empty(); }

    void add(typename Base<T>::sptr fp) {
      filters_.push_back(fp);
    }
    virtual void init(boost::posix_time::ptime pt, const T& x) {
      x_= x;
      for (auto filter : filters_)
        filter->init(pt, x);
    }
    virtual const T& update(boost::posix_time::ptime pt, const T& x) {
      x_= x;
      for (auto filter : filters_)
        x_ = filter->update(pt, x_);
      return x_;
    }
    virtual const T& x() const {
      return filters_.empty() ? x_ : filters_.back()->x();
    }
    virtual bool isInEquilibrium() const {
      for (auto filter : filters_)
        if (not filter->isInEquilibrium()) return false;
      return true;
    }
  private:
    std::vector<typename Base<T>::sptr> filters_;
    T x_;
  } ;
}
#endif // _FILTER_HPP_cm101017_
