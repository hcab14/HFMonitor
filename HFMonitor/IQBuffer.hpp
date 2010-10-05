// -*- C++ -*-
// $Id$
#ifndef _IQ_BUFFER_HPP_cm101004_
#define _IQ_BUFFER_HPP_cm101004_

#include <iostream>
#include <iterator>
#include <algorithm>
#include <vector>
#include <complex>
#include <stdexcept>
#include <boost/bind.hpp>

namespace Internal {
  // modulo counter without using the % operation
  template<typename T>
  class ModuloCounter {
  public:
    explicit ModuloCounter(T m) : m_(m), t_(0) {}    

    T m() const { return m_; }
    operator T() const { return t_; }

    ModuloCounter& operator++() {
      const T tInc(t_+1);
      t_ = (tInc >= m_) ? T(0) : tInc;
      return *this;
    }
    ModuloCounter& operator--() {
      t_ = (t_ == T(0)) ? m_-1 : t_-1;
      return *this;
    }

  private:
    const T m_;
    T t_;
  } ;
}

class IQBuffer {
public:
  typedef std::complex<double> Complex;
  typedef std::vector<Complex> Samples;
  IQBuffer(size_t n,       // buffer length
	   double overlap) // overlap [0..1]
    : iqVec_(2*n,0)
    , n_(n)
    , m_(size_t((1.0-overlap)*n_+0.5))
    , lpi_(0)
    , counterModN_(n_)
    , counterModM_(m_)
    , counterModNM_(m_*n_) {
    if (overlap < 0.0 || overlap > 1.0)
      throw std::runtime_error("IQBuffer: overlap < 0.0 || overlap > 1.0");
  }

  virtual ~IQBuffer() {}

  // returns the remaining unprocessed samples
  Samples samples() const {
    Samples s;
    std::copy(iqVec_.begin()+lastI0(), iqVec_.begin()+lastI1(), 
	      std::back_inserter(s));
    return s;
  }

  size_t n() const { return n_; }
  size_t m() const { return m_; }
  double overlap() const { return 1.0-double(m_)/double(n_); }
 
  // insert a range of samples
  void insert(Samples::const_iterator begin, Samples::const_iterator end) {
    void (IQBuffer::*fp)(Complex)(&IQBuffer::insert);
    std::for_each(begin, end, boost::bind(fp, this, _1));
  }
  // insert a single sample
  void insert(Complex c) {
    if (counterModM_ == 0) {
      procIQ(iqVec_.begin()+counterModN_, iqVec_.begin()+counterModN_+n_);
      lpi_ = (counterModN_+m_) % n_;
    }
    iqVec_[counterModN_] = iqVec_[counterModN_+n_] = c;
    ++counterModN_;
    ++counterModM_;
    ++counterModNM_;
  }

  virtual void procIQ(Samples::const_iterator i0, 
		      Samples::const_iterator i1) {
    std::cout << "*** procIQ ";
    std::copy(i0, i1, 
	      std::ostream_iterator<std::complex<double> >(std::cout, " "));
    std::cout << "***" << std::endl;
  }

protected:
private:
  // returns first index of not yet processed samples
  size_t lastI0() const {
    return lpi_;
  }
  // returns last index of not yet processed samples
  size_t lastI1() const {
    return (counterModN_+n_ - lastI0() > n_ 
	    ? size_t(counterModN_) 
	    : counterModN_+n_);
  }

  Samples iqVec_;
  const size_t n_;
  const size_t m_;
  size_t lpi_;
  Internal::ModuloCounter<size_t> counterModN_;
  Internal::ModuloCounter<size_t> counterModM_;
  Internal::ModuloCounter<size_t> counterModNM_;
} ;


#endif // _IQ_BUFFER_HPP_cm101004_
