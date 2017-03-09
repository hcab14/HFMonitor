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
    , _f_max(_center_freq + 0.5*_sample_rate) {
    ASSERT_THROW(size > 0);
  }

  virtual ~fft_data_base() {}

  size_t size() const { return _size; }
  double center_freq() const { return _center_freq*_freq_correction; }
  double sample_rate() const { return _sample_rate*_freq_correction; }
  double f_min() const { return _f_min*_freq_correction; }
  double f_max() const { return _f_max*_freq_correction; }
  double delta_f() const { return f_max()-f_min(); }

  size_t freq2index(double freq) const {
    const double df = std::max(0.0, freq-f_min());
    return std::min(size()-1, size_t(0.5 + size() * df/delta_f()));
  }
  double index2freq(size_t index) const {
    return f_min() + double(index)/double(size()) * delta_f();
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
  void update_size(size_t n) {
    ASSERT_THROW(n > 0);
    _size = n;
  }
  double freq_corr() const { return _freq_correction; }
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

  int find_peak(float f0, float f1) const {
    int i0 = freq2index(f0);
    int i1 = freq2index(f1);
    ASSERT_THROW(i0 < i1);
    uint16_t i_max=0;
    volk_32f_index_max_16u(&i_max, &begin()[0]+i0, i1-i0);
    return i0 + i_max;
  }
  float estimate_peak_freq(int i_max) const {
    const int i0 = std::max(       i_max-3, 0);
    const int i1 = std::min(size_t(i_max+4), size()-1);

    float sum_w(0), sum_wx(0);
    for (const_iterator i=begin()+i0, iend=begin()+i1; i!=iend; ++i) {
      const float f = index2freq(std::distance(begin(), i));
      const float w = std::pow(10.0f, *i*0.1);
      sum_w  += w;
      sum_wx += w*f;
    }
    return sum_wx/sum_w;
  }
  float get_noise_floor(float f0, float f1, float exclusion_value) const {
    const int i0 = freq2index(f0);
    const int i1 = freq2index(f1);
    ASSERT_THROW(i0 < i1);
    float noise_floor=0;
    volk_32f_s32f_calc_spectral_noise_floor_32f(&noise_floor, &begin()[0]+i0, exclusion_value, i1-i0);
    return noise_floor;
  }

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
