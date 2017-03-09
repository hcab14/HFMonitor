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
#ifndef _FFT_HPP_cm100823_
#define _FFT_HPP_cm100823_

#include <complex>
#include <vector>
#include <fftw3.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include "aligned_vector.hpp"

namespace FFT {
  namespace WindowFunction {
    /// rectangular window function
    template<typename T>
    class base {
    public:
      base(size_t n)
        :  n_(n) {}
      virtual ~base() {}
      T gain() const {
        T gain_val = 0;
        for (size_t i=0; i<n_; ++i)
          gain_val += (*this)(i);
        return gain_val/n_;
      }
      T operator()(size_t i) const { return eval(i); }
    protected:
      virtual T eval(size_t) const = 0;
    private:
      size_t n_;
    };

    template<typename T>
    struct Rectangular : public base<T> {
      Rectangular(size_t n)
        : base<T>(n) {}
      virtual T eval(size_t) const { return T(1); }
    } ;
    /// Hanning window function
    template<typename T>
    class Hanning : public base<T> {
    public:
      Hanning(size_t n)
        : base<T>(n)
        , norm_(1./double(n-1)){}
    protected:
      virtual T eval(size_t u) const {
        return 0.5*(1.0-std::cos(2.*M_PI*u*norm_));
      }
    private:
      const double norm_;
    } ;
    /// Hamming window function
    template<typename T>
    class Hamming : public WindowFunction::base<T> {
    public:
      Hamming(size_t n)
        : base<T>(n)
        , norm_(1./double(n-1)) {}
    protected:
      virtual T eval(size_t u) const {
        return 0.54 - 0.46*std::cos(2.*M_PI*u*norm_);
      }
    private:
      const double norm_;
    } ;
    /// Gaussian window function
    template<typename T>
    class Gaussian : public base<T> {
    public:
      Gaussian(size_t n, T sigma)
        : base<T>(n)
        , norm_(1./(0.5*sigma*(n-1)))
        , mean_(0.5*(n-1)) {}
    protected:
      virtual T eval(size_t u) const {
        return std::exp(-0.5*std::pow((u-mean_)*norm_, 2));
      }
    private:
      const double norm_;
      const double mean_;
    } ;
    /// Blackman window function
    template<typename T>
    class Blackman : public base<T> {
    public:
      Blackman(size_t n, T alpha=0.16)
        : base<T>(n)
        , a0_(0.5*(1-alpha))
        , a1_(0.5)
        , a2_(0.5*alpha)
        , norm_(1./double(n-1)) {}
      virtual T eval(size_t u) const {
        const T t(2.*M_PI*u*norm_);
        return a0_ - a1_*std::cos(t) + a2_*std::cos(2*t);
      }
    private:
      const T a0_, a1_, a2_;
      const double norm_;
    } ;
  } // namespace WindowFunction

  namespace Internal {
    template<typename>
    struct FFTWTraits ;

#define MAKE_TRAITS(P, T)                                               \
    template<>                                                          \
    struct FFTWTraits<T> {                                              \
      typedef T value_type;                                             \
      typedef FFTW_CONCAT(P,_complex) complex_type;                     \
      typedef FFTW_CONCAT(P,_plan) Plan;                                \
                                                                        \
      static complex_type* malloc(size_t n) {                           \
        return (complex_type*)FFTW_CONCAT(P,_malloc)(sizeof(complex_type)*n); \
      }                                                                 \
      static void free(complex_type* a) { FFTW_CONCAT(P,_free)(a); }    \
      static Plan plan_dft_1d(size_t n, complex_type* in, complex_type* out, \
                              int sign, unsigned flags) {               \
        const int nthreads(4);                                          \
        fftw_plan_with_nthreads(nthreads);                              \
        return FFTW_CONCAT(P,_plan_dft_1d)(n, in, out, sign, flags);    \
      }                                                                 \
      static void execute(Plan p) {                                     \
        FFTW_CONCAT(P,_execute)(p);                                     \
      }                                                                 \
      static void destroy_plan(Plan p) {                                \
        FFTW_CONCAT(P,_destroy_plan)(p);                                \
      }                                                                 \
    }

    MAKE_TRAITS(fftwf, float);
    MAKE_TRAITS(fftw,  double);
    MAKE_TRAITS(fftwl, long double);

  } // namespace Internal

  /// interface to FFTW (double or single precision)
  template<typename T>
  class FFTWTransform : public Internal::FFTWTraits<T> {
  public:
    typedef boost::shared_ptr<FFTWTransform<T> > sptr;
    typedef Internal::FFTWTraits<T> Traits;
    typedef typename Traits::complex_type fftw_complex_type;
    typedef typename std::complex<T> complex_type;
    typedef typename Traits::Plan Plan;

    typedef aligned_vector<complex_type> vector_type;

    using Traits::plan_dft_1d;
    using Traits::execute;
    using Traits::destroy_plan;

    FFTWTransform(size_t n, int sign, unsigned flags)
      : in_(n)
      , out_(n)
      , sign_(sign)
      , flags_(flags)
      , plan_(plan_dft_1d(n, (fftw_complex_type*)&in_[0], (fftw_complex_type*)&out_[0], sign, flags))
      , normalizationFactor_(T(1)/T(n)) {}

    ~FFTWTransform() {
      destroy_plan(plan_);
    }

    /// resize
    void resize(size_t n) {
      if (n != size()) {
        in_.resize(n);
        out_.resize(n);
        destroy_plan(plan_);
        plan_ = plan_dft_1d(in_.size(), (fftw_complex_type*)&in_[0], (fftw_complex_type*)&out_[0], sign_, flags_);
        // normalizationFactor_ has to be updated by hand
      }
    }

    /// norm of window
    double normWindow() const { return in_.size() * normalizationFactor_; }

    /// transform the input vector \c v using the window function \c window_fcn
    template<typename V,
             typename W>
    void transformVector(const V& v,
                         const W& window_fcn) {
      transformRange(v.begin(), v.end(), window_fcn);
    }
    /// transform a vector range specified by iterators \c i0 and \c i1 using the window function \c window_fcn
    template<typename IT,
             typename W>
    void transformRange(IT i0,
                        IT i1,
                        const W& window_fcn) {
      const size_t length(std::distance(i0, i1));
      if (length != size())
        resize(length);
      for (size_t i=0; i<length; ++i)
        in_[i] = typename IT::value_type::value_type(window_fcn(i)) * *i0++;
      normalizationFactor_= T(1)/T(length)/window_fcn.gain();
      execute(plan_);
    }
    /// transform
    void transform() {
      normalizationFactor_= 1;
      execute(plan_);
    }

    /// size of the FFT transform
    size_t size() const { return in_.size(); }

    float normalization_factor() const { return normalizationFactor_; }

    /// get the result of a single bin
    /// amplitude is corrected for the spread due to the used window function
    complex_type getBin(size_t u) const {
      return normalizationFactor_ * out_[u];
    }

    vector_type& in() { return in_; }
    const vector_type& out() const { return out_; }

    /// access for input array (non-const)
    complex_type& in (size_t index) { return in_[index]; }
    /// access for output array (non-const)
    complex_type& out(size_t index) { return out_[index]; }

    /// access for input array (const)
    const complex_type& in (size_t index) const { return in_[index]; }
    /// access for output array (const)
    const complex_type& out(size_t index) const { return out_[index]; }

  protected:
  private:
    vector_type              in_;   /// input FFTW array
    vector_type             out_;  /// output FFTW array
    int                    sign_; /// sign of transform
    unsigned              flags_; /// FFTW flags
    Plan                   plan_; /// FFTW plan
    T       normalizationFactor_; /// normalization factor (depends on used window function)
  } ;

  class FFTWInitThreads {
  public:
    FFTWInitThreads() {
      fftw_init_threads();
    }
    ~FFTWInitThreads() {
      fftw_cleanup_threads();
    }
  protected:
  private:
  } ;

  struct fftw_setup {
    typedef FFTWTransform<float>::sptr sptr;
    enum {
      FORWARD  = FFTW_FORWARD,  // +1
      BACKWARD = FFTW_BACKWARD  // -1
    };
    sptr make(size_t n, int sign) {
      return boost::make_shared<sptr::element_type>(n, sign, FFTW_ESTIMATE);
    }
  } ;
} // namespace FFT

#ifdef USE_CUDA
#  include "CUFFT.ipp"
#endif

#endif // _FFT_HPP_cm100823_
