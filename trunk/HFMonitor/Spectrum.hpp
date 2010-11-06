// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _SPECTRUM_HPP_cm101026_
#define _SPECTRUM_HPP_cm101026_

#include <complex>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <boost/noncopyable.hpp>
#include <boost/foreach.hpp>

#include "FFT.hpp"

class SpectrumBase : private boost::noncopyable {
public:
  typedef std::complex<double> Complex;

  SpectrumBase(double sampleRate,
               double centerFrequency)
    : sampleRate_(sampleRate)
    , centerFrequency_(centerFrequency) {}
  virtual ~SpectrumBase() {}

  double sampleRate() const { return sampleRate_; }
  double centerFrequency() const { return centerFrequency_; }

  virtual size_t size()                    const = 0;
  virtual Complex operator[](size_t index) const = 0;

  size_t freq2index(double qrg_Hz) const { // get the nearest bin index
    const int n(size());
    const int i(round(double(n)*(qrg_Hz - centerFrequency()) / sampleRate()));
    if (i >= -n/2 && i < n/2)
      return (i>=0) ? i : n+i;
    else 
      throw std::runtime_error("freq2index failed");
  }
  
  double index2freq(size_t index) const {
    const int n(size());
    return centerFrequency() + sampleRate() * (int(index)>=n/2 ? -n+int(index) : int(index)) / double(n);
  }  
private:
  static int round(double d) { return int(d+((d>=0.0) ? .5 : -.5)); }

  double sampleRate_;
  double centerFrequency_;
} ;

template<typename T>
class FFTWSpectrum : public SpectrumBase {
public:
  FFTWSpectrum(const FFT::FFTWTransform<T>& fftw,
               double sampleRate,
               double centerFrequency)
    : SpectrumBase(sampleRate, centerFrequency)
    , fftw_(fftw) {}
  virtual ~FFTWSpectrum() {}

  virtual size_t size() const { return fftw_.size(); }
  virtual std::complex<double> operator[](size_t index) const { return fftw_.getBin(index); }  

protected:
private:
  const FFT::FFTWTransform<T>& fftw_;
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

  frequency_vector(freq_type fmin, freq_type fmax)
    : fmin_(fmin), fmax_(fmax) {}
  
  template<typename FUNCTION>
  frequency_vector(freq_type fmin, freq_type fmax,
                   const SpectrumBase& s, FUNCTION func) 
    : fmin_(fmin), fmax_(fmax) { fill(s, func); }
  
  freq_type fmin() const { return fmin_; }
  freq_type fmax() const { return fmax_; }
  
  template<typename FUNCTION>
  frequency_vector<T> fill(const SpectrumBase& s, const FUNCTION& func) {
    v_.clear();
    const size_t i0(s.freq2index(fmin()));
    const size_t i1(s.freq2index(fmax()));
    fmin_ = s.index2freq(i0);
    fmax_ = s.index2freq(i1);
    for (size_t u=i0; u<=i1; ++u) 
      v_.push_back(value_type(s.index2freq(u), func(s[u])));
    return *this;
  }
  template<typename FUNCTION>
  frequency_vector<T> apply(const FUNCTION& func) {
    BOOST_FOREACH(value_type& x, v_)
      x.second= func(x.second);
    return *this;
  }

  size_t freq2index(freq_type f) const {
    if (f >= fmin_ && f <= fmax_)
      return size_t(0.5+(f-fmin_)/(fmax_-fmin_) * v_.size());
    else
      throw std::runtime_error("freq2index: f out of range");
  }
  
  const value_type& operator[](unsigned index) const { return v_[index]; }
  void clear() { v_.clear(); }
  size_t size() const { return v_.size(); }
  bool empty() const { return v_.empty(); }
  iterator       begin()       { return v_.begin(); }
  const_iterator begin() const { return v_.begin(); }
  iterator       end()       { return v_.end(); }
  const_iterator end() const { return v_.end(); }

  static bool cmpFreq(const value_type& x1, const value_type& x2) {
    return x1.first < x2.first;
  }
  static bool cmpSecond(const value_type& x1, const value_type& x2) {
    return x1.second < x2.second;
  }

  frequency_vector<T>& operator+=(const frequency_vector<T>& v) {
    if (size() != v.size())
      throw std::runtime_error("frequency_vector<T>::operator+= (size() != v.size())");
    const_iterator j(v.begin());
    for (iterator i(begin()); i!=end(); ++i,++j) {
      if (i->first != j->first) 
        throw std::runtime_error("frequency_vector<T>::operator+= (i->first != j->first)");
      i->second += j->second;
    }
    return *this;
  }
  frequency_vector<T>& operator-=(const frequency_vector<T>& v) {
    if (size() != v.size())
      throw std::runtime_error("frequency_vector<T>::operator-= (size() != v.size())");
    const_iterator j(v.begin());
    for (iterator i(begin()); i!=end(); ++i,++j) {
      if (i->first != j->first)
        throw std::runtime_error("frequency_vector<T>::operator-= (i->first != j->first)");
      i->second -= j->second;
    }
    return *this;
  }
  frequency_vector<T>& operator*=(double f) {
    for (iterator i(begin()); i!=end(); ++i)
      i->second *= f;
    return *this;
  }
  friend frequency_vector<T> operator*(const frequency_vector<T>& v, double f) {
    frequency_vector<T> r(v); r*=f;
    return r;
  }
  friend frequency_vector<T> operator*(double f, const frequency_vector<T>& v) {    
    frequency_vector<T> r(v); r*=f;
    return r;
  }
  frequency_vector<T>& operator*=(const frequency_vector<T>& v) {
    if (size() != v.size()) 
      throw std::runtime_error("frequency_vector<T>::operator*= (size() != p.size())");
    const_iterator j(v.begin());
    for (iterator i(begin()); i!=end(); ++i,++j) {
      if (i->first != j->first) 
        throw std::runtime_error("frequency_vector<T>::operator*= (i->first != j->first)");
      i->second *= j->second;
    }
    return *this;
  }
  friend frequency_vector<T> operator*(const frequency_vector<T>& v1,
                                       const frequency_vector<T>& v2) {
    frequency_vector<T> r(v1); r*=v2;
    return r;
  }
  
  friend frequency_vector<T> sqrt(frequency_vector<T> v) {
    return v.apply(std::sqrt<T>);
  }

protected:
private:
  freq_type fmin_;
  freq_type fmax_;
  vector_type v_;
} ;
#endif // _SPECTRUM_HPP_cm101026_
