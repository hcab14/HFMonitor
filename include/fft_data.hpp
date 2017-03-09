// -*- C++ -*-
#ifndef _FFT_DATA_HPP_cm170309_
#define _FFT_DATA_HPP_cm170309_

#include <volk/volk.h>

#include <boost/shared_ptr.hpp>

#include "aligned_vector.hpp"

class fft_data_base {
public:
  fft_data_base(size_t size,
                double sample_rate,
                double center_freq,
                double offset_ppb)
    : _size(size)
    , _sample_rate(sample_rate)
    , _center_freq(center_freq)
    , _freq_correction(1. - 1e-9*offset_ppb)
    , _f_min(_center_freq - 0.5*_sample_rate)
    , _f_max(_center_freq + 0.5*_sample_rate) {}

  virtual ~fft_data_base() {}

  size_t size() const { return _size; }
  double center_freq() const { return _center_freq*_freq_correction; }
  double sample_rate() const { return _sample_rate*_freq_correction; }
  double f_min() const { return _f_min*_freq_correction; }
  double f_max() const { return _f_max*_freq_correction; }

  size_t freq2index(double freq) const {
    return size_t(0.5 + size() * (freq-f_min()) / (f_max()-f_min()));
  }
  double index2freq(size_t index) const {
    return f_min() + double(index)/double(size()) * (f_max()-f_min());
  }

  void update(double sample_rate,
	      double center_freq,
	      double offset_ppb) {
    _sample_rate     = sample_rate;
    _center_freq     = center_freq;
    _freq_correction = 1. - 1e-9*offset_ppb;
  }
protected:
  // this is used in derived classes
  void update_size(size_t n) { _size = n; }

private:
  size_t _size;
  double _sample_rate;     // Hz
  double _center_freq;     // Hz
  double _freq_correction; // multiplicative frequency correction
  double _f_min;           // Hz
  double _f_max;           // Hz
} ;

class fft_power_spectrum : public fft_data_base {
public:
  typedef boost::shared_ptr<fft_power_spectrum> sptr;
  typedef aligned_vector<std::complex<float> > complex_vector_type;
  typedef aligned_vector<float> vector_type;
  typedef vector_type::iterator iterator;
  typedef vector_type::const_iterator const_iterator;

  fft_power_spectrum(size_t size,
                     double sample_rate,
                     double center_freq,
                     double offset_ppb)
    : fft_data_base(size, sample_rate, center_freq, offset_ppb)
    , _in(size)
    , _ps(size) {}

  virtual ~fft_power_spectrum() {}

  template<typename FFT>
  void insert(const FFT& fft) {
    resize(fft.size());
    auto const& fft_out(fft.out());
    const int mid = (size()+1)/2;
    std::copy(fft_out.end()-mid, fft_out.end(),     _in.begin());
    std::copy(fft_out.begin(),   fft_out.end()-mid, _in.begin()+mid);
    volk_32fc_s32f_power_spectrum_32f(&_ps[0], &_in[0],1.0f/ fft.normalization_factor(), size());
  }

  const_iterator begin() const { return _ps.begin(); }
  const_iterator end()   const { return _ps.end(); }

  iterator begin()  { return _ps.begin(); }
  iterator end()    { return _ps.end(); }
  
  float operator[](size_t i) const { return _ps[i]; }
  float& operator[](size_t i) { return _ps[i]; }

protected:
  void resize(size_t n) {
    if (_in.size() == n && _ps.size() == n)
      return;
    update_size(n);
    _in.resize(n);
    _ps.resize(n);
  }
private:
  complex_vector_type _in;
  vector_type         _ps;
} ;

#endif // _FFT_DATA_HPP_cm170309_
