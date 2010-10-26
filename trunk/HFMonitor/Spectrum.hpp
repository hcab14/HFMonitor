// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _SPECTRUM_HPP_cm101026_
#define _SPECTRUM_HPP_cm101026_

#include <complex>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <boost/noncopyable.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

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

  size_t freq2Index(double qrg_Hz) const { // get the nearest bin index
    const int n(size());
    const int i(round(double(n)*(qrg_Hz - centerFrequency()) / sampleRate()));
    if (i >= -n/2 && i < n/2)
      return (i>=0) ? i : n+i;
    else 
      throw std::runtime_error("freq2Index failed");
  }
  
  double index2Freq(size_t index) const {
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

// class FreqStrength {
// public:
//   FreqStrength(double freq, double strength)
//     : freq_(freq)
//     , strength_(strength) {}
  
//   double freq() const { return freq_; }
//   double strength() const { return strength_; }
//   double& strength() { return strength_; }
  
//   static bool cmpStrength(const FreqStrength& fs1, 
//                           const FreqStrength& fs2) {
//     return fs1.strength() < fs2.strength();
//   }
//   static bool cmpFreq(const FreqStrength& fs1, 
//                       const FreqStrength& fs2) {
//     return fs1.freq() < fs2.freq();
//   }
// private:
//   double freq_;
//   double strength_;
// } ;

class PowerSpectrum {
public:
  typedef std::pair<double, double> FreqStrength;
  typedef std::vector<FreqStrength> Vec;
  typedef Vec::iterator       iterator;
  typedef Vec::const_iterator const_iterator;
  typedef boost::posix_time::ptime ptime;

  PowerSpectrum(double fMin, double fMax)
    : fMin_(fMin)
    , fMax_(fMax) {}

  PowerSpectrum(double fMin, double fMax, const SpectrumBase& s)
    : fMin_(fMin)
    , fMax_(fMax) {
    fill(s);
  }
  
  double fMin() const { return fMin_; }
  double fMax() const { return fMax_; }

  void fill(const SpectrumBase& s) {
    ps_.clear();
    const size_t i0(s.freq2Index(fMin_));
    const size_t i1(s.freq2Index(fMax_));
    for (size_t u=i0; u<=i1; ++u) 
      ps_.push_back(FreqStrength(s.index2Freq(u), std::abs(s[u])));
  }
  const FreqStrength& operator[](unsigned index) const { return ps_[index]; }
  void clear() { ps_.clear(); }
  size_t size() const { return ps_.size(); }
  bool empty() const { return ps_.empty(); }
  iterator       begin()       { return ps_.begin(); }
  const_iterator begin() const { return ps_.begin(); }
  iterator       end()       { return ps_.end(); }
  const_iterator end() const { return ps_.end(); }

  static bool cmpFreq(const FreqStrength& fs1,
                      const FreqStrength& fs2) {
    return fs1.first < fs2.first;
  }
  static bool cmpStrength(const FreqStrength& fs1,
                          const FreqStrength& fs2) {
    return fs1.second < fs2.second;
  }

  PowerSpectrum& operator+=(const PowerSpectrum& ps) {
    if (size() != ps.size())
      throw 1; // TODO
    const_iterator j(ps.begin());
    for (iterator i(begin()); i!=end(); ++i,++j) {
      if (i->first != j->first) throw 1; // TODO
      i->second += j->second;
    }
    return *this;
  }
  PowerSpectrum& operator-=(const PowerSpectrum& ps) {
    if (size() != ps.size())
      throw 1; // TODO
    const_iterator j(ps.begin());
    for (iterator i(begin()); i!=end(); ++i,++j) {
      if (i->first != j->first) throw 1; //TODO
      i->second -= j->second;
    }
    return *this;
  }
  PowerSpectrum& operator*=(double f) {
    for (iterator i(begin()); i!=end(); ++i)
      i->second *= f;
    return *this;
  }
  friend PowerSpectrum operator*(const PowerSpectrum& p,
                                 double f) {
    PowerSpectrum r(p); r*=f;
    return r;
  }
  friend PowerSpectrum operator*(double f,
                                 const PowerSpectrum& p) {    
    PowerSpectrum r(p); r*=f;
    return r;
  }

  size_t pgmSize() const {
    return sizeof(ptime) + 2*sizeof(float) + ps_.size();
  }
  std::string pgmLine(ptime t) const {
    std::string line;
    std::copy((char*)&t, (char*)&t+sizeof(t), std::back_inserter(line));
    if (ps_.empty())
      return line.append(2*sizeof(float), char(0));

    const_iterator iMin(std::min_element(ps_.begin(), ps_.end(), cmpStrength));
    const float slMin(std::log10(iMin->second));
    std::copy((char*)&slMin, (char*)&slMin+sizeof(slMin), std::back_inserter(line));

    const_iterator iMax(std::max_element(ps_.begin(), ps_.end(), cmpStrength));
    const float slMax(std::log10(iMax->second));
    std::copy((char*)&slMax, (char*)&slMax+sizeof(slMax), std::back_inserter(line));

    for (const_iterator i(ps_.begin()); i!=ps_.end(); ++i)
      line.push_back((unsigned char)((std::log10(i->second) - slMin)/(slMax-slMin)*255));
    
    return line;
  }

private:
  const double fMin_;
  const double fMax_;
  Vec ps_;
} ;

#endif // _SPECTRUM_HPP_cm101026_
