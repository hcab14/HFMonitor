// -*- C++ -*-
// $Id$
#ifndef _IQ_BUFFER_HPP_cm101004_
#define _IQ_BUFFER_HPP_cm101004_

#include <iostream>
#include <iterator>
#include <vector>
#include <complex>

class IQBuffer {
public:
  typedef std::complex<double> Complex;
  typedef std::vector<Complex> Samples;
  IQBuffer(size_t n,       // buffer length
	   double overlap) // overlap [0..1]
    : iqVec_(2*n,0)
    , n_(n)
    , m_((1.0-overlap)*n_+0.5)
    , lastProcessedIndex_(0)
    , counter_(0) {}

  // Samples samples() const {
  //   Samples s;
  //   const size_t i(counter_ % n_);
  //   for (size_t u(i); u<n_; ++u)
  //     s.push_back(iqVec_[u]);
  //   return s;
  // }
  size_t counter() const { return counter_; }
  size_t n() const { return n_; }
  size_t m() const { return m_; }
  double overlap() const { return 1.0-double(m_)/double(n_); }
  
  void insert(Complex c) {
    const size_t i(counter_ % n_);
    const size_t lastI0((m_*(1+(counter_)/m_) % n_));
    const size_t lastI1(i+n_);
    std::cout << "counter_,i= " << counter_ << " " << i << " " << lastI0 << " : " ;
    std::copy(iqVec_.begin(), iqVec_.end(), std::ostream_iterator<std::complex<double> >(std::cout, " "));
    std::cout << std::endl;
    lastIQ(iqVec_.begin()+lastI0, iqVec_.begin()+lastI1);
    if (counter_ % m_ == 0) {
      // std::cout << "*** procIQ: " << i << "-" << i+n_ << std::endl;;
      procIQ(iqVec_.begin()+i, iqVec_.begin()+i+n_);
      lastProcessedIndex_ = i+n_;
    }
    iqVec_[i] = iqVec_[i+n_] = c;
    counter_ = (1+counter_ == m_*n_) ? 0 : 1+counter_;
  }

  virtual void procIQ(Samples::const_iterator i0, Samples::const_iterator i1) {
    std::cout << "*** procIQ ";
    std::copy(i0,i1, std::ostream_iterator<std::complex<double> >(std::cout, " "));
    std::cout << std::endl;
  }
  virtual void lastIQ(Samples::const_iterator i0, Samples::const_iterator i1) {
    std::cout << "    lastIQ ";
    std::copy(i0,i1, std::ostream_iterator<std::complex<double> >(std::cout, " "));
    std::cout << std::endl;
  }
protected:
private:
  Samples iqVec_;
  size_t n_;
  size_t m_;
  size_t lastProcessedIndex_;
  size_t counter_;
} ;


#endif // _IQ_BUFFER_HPP_cm101004_
