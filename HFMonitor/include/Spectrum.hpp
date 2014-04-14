// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2013 Christoph Mayer
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
#ifndef _SPECTRUM_HPP_cm101026_
#define _SPECTRUM_HPP_cm101026_

#include <complex>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <boost/noncopyable.hpp>
#include <boost/foreach.hpp>

#include "logging.hpp"

class SpectrumBase : private boost::noncopyable {
public:
  typedef std::complex<double> complex_type;

  SpectrumBase(double sampleRate,
               double centerFrequency)
    : sampleRate_(sampleRate)
    , centerFrequency_(centerFrequency) {}
  virtual ~SpectrumBase() {}

  double sampleRate() const { return sampleRate_; }
  double centerFrequency() const { return centerFrequency_; }

  virtual size_t size()                         const = 0;
  virtual complex_type operator[](size_t index) const = 0;
  virtual double normWindow()                   const = 0;

  size_t freq2index(double qrg_Hz) const { // get the nearest bin index
    // clamp to f0 +- fs/2
    qrg_Hz = std::min(qrg_Hz, centerFrequency() + sampleRate()/2);
    qrg_Hz = std::max(qrg_Hz, centerFrequency() - sampleRate()/2);
    const int n(size());
    int i(round(double(n)*(qrg_Hz - centerFrequency()) / sampleRate()));
    ASSERT_THROW(-n/2 <= i && i <= n/2);
    // make sure that +f0 and -f0 do not get the same index
    if (i == n/2) i -= 1;
    return (i>=0) ? i : n+i;
  }
  
  double index2freq(size_t index) const {
    const int n(size());
    return centerFrequency() + sampleRate() * (int(index) < n/2
                                               ?    int(index)
                                               : -n+int(index))/double(n);
  }  
private:
  static int round(double d) { return int(d+((d>=0.0) ? .5 : -.5)); }

  double sampleRate_;
  double centerFrequency_;
} ;

template<typename FFT_TYPE>
class FFTSpectrum : public SpectrumBase {
public:
  FFTSpectrum(const FFT_TYPE& fftw,
              double sampleRate,
              double centerFrequency)
    : SpectrumBase(sampleRate, centerFrequency)
    , fftw_(fftw) {}
  virtual ~FFTSpectrum() {}

  virtual size_t size() const { return fftw_.size(); }
  virtual complex_type operator[](size_t index) const { return fftw_.getBin(index); }  
  virtual double normWindow() const { return fftw_.normWindow(); }

protected:
private:
  const FFT_TYPE& fftw_;
  double sampleRate_;
  double centerFrequency_;
} ;

template<typename T>
class frequency_vector {
public:
  typedef double freq_type;
  typedef typename std::pair<freq_type, T> value_type;
  typedef typename std::vector<value_type> vector_type;
  typedef typename vector_type::iterator iterator;
  typedef typename vector_type::const_iterator const_iterator;

  frequency_vector(freq_type fmin=freq_type(0), 
                   freq_type fmax=freq_type(1))
    : fmin_(fmin), fmax_(fmax) {}
  
  frequency_vector(freq_type fmin, freq_type fmax, size_t size, const T& value)
    : fmin_(fmin),
      fmax_(fmax),
      v_(size) {
    for (unsigned u(0); u<size; ++u)
      v_[u]= std::make_pair(fmin+(fmax-fmin)/(size-1)*u, value);
  }
  
  template<typename FUNCTION>
  frequency_vector(freq_type fmin, freq_type fmax,
                   const SpectrumBase& s, FUNCTION func, double offset_ppb=0.)
    : fmin_(fmin), fmax_(fmax) { fill(s, func, offset_ppb); }

  frequency_vector(const frequency_vector& f) 
    : fmin_(f.fmin_), fmax_(f.fmax_), v_(f.v_) {} 

  frequency_vector& operator=(const frequency_vector& f) {
    if (&f != this) {
      frequency_vector tmp(f);
      std::swap(fmin_, tmp.fmin_);
      std::swap(fmax_, tmp.fmax_);
      std::swap(v_,    tmp.v_);
    }
    return *this;
  }
 
  freq_type fmin() const { return fmin_; }
  freq_type fmax() const { return fmax_; }
  freq_type deltaf() const { return (v_.size() > 1) ? 0.0 : (fmax() - fmin()) / (size()-1); }

  template<typename FUNCTION>
  frequency_vector<T>& fill(const SpectrumBase& s, const FUNCTION& func, double offset_ppb=0.) {
    const double correction_factor(1. - 1e-9*offset_ppb);

    // find nearest frequency bin
    const size_t i0(s.freq2index(fmin()));
    fmin_ = s.index2freq(i0);

    // find nearest frequency bin
    const size_t i1(s.freq2index(fmax()));
    fmax_ = s.index2freq(i1);

    if (i0 <= i1) {
      const size_t n(1+i1-i0);
      if (v_.size() != n) v_.resize(n);
      for (size_t u(i0); u<=i1; ++u) {
        const double f(s.index2freq(u));
        const size_t ucal(s.freq2index(correction_factor*f));
        v_[u -i0    ] = std::make_pair(f, func(s[ucal]));
      }
    } else {
      const size_t n0(s.size()); // length of FFT spectrum
      const size_t n(1 + n0-i0 + i1);
      if (v_.size() != n) v_.resize(n);
      for (size_t u(i0); u<n0; ++u) {
        const double f(s.index2freq(u));
        const size_t ucal(s.freq2index(correction_factor*f));
        v_[u -i0    ] = std::make_pair(f, func(s[ucal]));
      }
      for (size_t u(0); u<=i1; ++u) {
        const double f(s.index2freq(u));
        const size_t ucal(s.freq2index(correction_factor*f));
        v_[n0-i0 + u] = std::make_pair(f, func(s[ucal]));
      }
    }
    return *this;
  }
  template<typename FUNCTION>
  frequency_vector<T> apply(const FUNCTION& func) {
    BOOST_FOREACH(value_type& x, v_)
      x.second= func(x.second);
    return *this;
  }

  size_t freq2index(freq_type f, bool do_throw=true) const {
    if (do_throw) {
      ASSERT_THROW(f >= fmin_ && f <= fmax_);
    } else {
      f = std::min(std::max(f, fmin_), fmax_);
    }
    return size_t(0.5+(f-fmin_)/(fmax_-fmin_) * (v_.size()-1));
  }
  
  const value_type& operator[](unsigned index) const { return v_[index]; }

  void clear() { v_.clear(); }

  size_t size() const { return v_.size(); }

  bool empty() const { return v_.empty(); }

  iterator       begin()       { return v_.begin(); }
  const_iterator begin() const { return v_.begin(); }

  iterator       end()       { return v_.end(); }
  const_iterator end() const { return v_.end(); }

  const value_type& front() const { return v_.front(); }
  value_type&       front()       { return v_.front(); }

  const value_type& back() const { return v_.back(); }
  value_type&       back()       { return v_.back(); }


  static bool cmpFreq(const value_type& x1, const value_type& x2) {
    return x1.first < x2.first;
  }
  static bool cmpSecond(const value_type& x1, const value_type& x2) {
    return x1.second < x2.second;
  }
  // addition
  frequency_vector<T>& operator+=(const frequency_vector<T>& v) {
    ASSERT_THROW(size() == v.size());
    const_iterator j(v.begin());
    for (iterator i(begin()), iend(end()); i!=iend; ++i,++j) {
      ASSERT_THROW(i->first == j->first);
      i->second += j->second;
    }
    return *this;
  }
  friend frequency_vector<T> operator+(const frequency_vector<T>& v1,
                                       const frequency_vector<T>& v2) {
    frequency_vector<T> r(v1); r+=v2;
    return r;
  }

  // subtraction
  frequency_vector<T>& operator-=(const frequency_vector<T>& v) {
    ASSERT_THROW(size() != v.size());
    const_iterator j(v.begin());
    for (iterator i(begin()), iend(end()); i!=iend; ++i,++j) {
      ASSERT_THROW(i->first == j->first);
      i->second -= j->second;
    }
    return *this;
  }
  friend frequency_vector<T> operator-(const frequency_vector<T>& v1,
                                       const frequency_vector<T>& v2) {
    frequency_vector<T> r(v1); r-=v2;
    return r;
  }

  // multiplication/division with scalar factor
  frequency_vector<T>& operator*=(double f) {
    BOOST_FOREACH(value_type& x, v_) x.second *= f;
    return *this;
  }
  frequency_vector<T>& operator/=(double f) {
    BOOST_FOREACH(value_type& x, v_) x.second /= f;
    return *this;
  }
  friend frequency_vector<T> operator*(const frequency_vector<T>& v, double f) {
    frequency_vector<T> r(v); r*=f;
    return r;
  }
  friend frequency_vector<T> operator/(const frequency_vector<T>& v, double f) {
    frequency_vector<T> r(v); r/=f;
    return r;
  }
  friend frequency_vector<T> operator*(double f, const frequency_vector<T>& v) {    
    frequency_vector<T> r(v); r*=f;
    return r;
  }

  // component-wise multiplication 
  frequency_vector<T>& operator*=(const frequency_vector<T>& v) {
    ASSERT_THROW(size() == v.size());
    const_iterator j(v.begin());
    for (iterator i(begin()), iend(end()); i!=iend; ++i,++j) {
      ASSERT_THROW(i->first == j->first);
      i->second *= j->second;
    }
    return *this;
  }
  friend frequency_vector<T> operator*(const frequency_vector<T>& v1,
                                       const frequency_vector<T>& v2) {
    frequency_vector<T> r(v1); r*=v2;
    return r;
  }
  // use sqrt_ (defined below) in order to avoid sqrt: double -> complex
  friend frequency_vector<T> sqrt(frequency_vector<T> v) {
    return v.apply(sqrt_);
  }

protected:
private:
  static T sqrt_(T x) { return sqrt(x); }

  freq_type fmin_;
  freq_type fmax_;
  vector_type v_;
} ;
#endif // _SPECTRUM_HPP_cm101026_
