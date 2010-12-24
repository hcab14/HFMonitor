// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_HPP_cm100823_
#define _FFT_HPP_cm100823_

#include <complex>
#include <fftw3.h>

namespace FFT {  
  namespace WindowFunction {
    template<typename T>
    struct Rectangular {
      T operator()(size_t u, size_t n) const { return T(1); }
    } ;
    template<typename T>
    struct Hanning {
      T operator()(size_t u, size_t n) const { 
        return 0.5*(1.0-std::cos(2.*M_PI*u/double(n-1)));
      }
    } ;
    template<typename T>
    struct Hamming {
      T operator()(size_t u, size_t n) const { 
        return 0.54 - 0.46*std::cos(2.*M_PI*u/double(n-1));
      }
    } ;
    template<typename T>
    class Gaussian {
    public:
      Gaussian(T sigma)
        : sigma_(sigma) {}
      T operator()(size_t u, size_t n) const { 
        return std::exp(-0.5*std::pow((u-0.5*(n-1)) / (0.5*sigma_*(n-1)), 2));
      }
    private:
      const T sigma_;
    } ;
    template<typename T>
    class Blackman {
    public:
      Blackman(T alpha=0.16)
        : a0_(0.5*(1-alpha))
        , a1_(0.5)
        , a2_(0.5*alpha) {}
      T operator()(size_t u, size_t n) const { 
        const T t(2.*M_PI*u/(n-1));
        return a0_ - a1_*std::cos(t) + a2_*std::cos(2*t);
      }
    private:
      const T a0_, a1_, a2_;
    } ;
  } // namespace WindowFunction

  namespace Internal {
    template<typename>
    struct FFTWTraits ;

#define MAKE_TRAITS(P, T)                                               \
    template<>                                                          \
    struct FFTWTraits<T> {                                              \
      typedef FFTW_CONCAT(P,_complex) Complex;                          \
      typedef FFTW_CONCAT(P,_plan) Plan;                                \
                                                                        \
      static Complex* malloc(size_t n) {                                \
        return (Complex*)FFTW_CONCAT(P,_malloc)(sizeof(Complex)*n);     \
      }                                                                 \
      static void free(Complex* a) { FFTW_CONCAT(P,_free)(a); }         \
      static Plan plan_dft_1d(size_t n, Complex* in, Complex* out,      \
                              int sign, unsigned flags) {               \
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

    template<typename T>
    class FFTWArray {
    public:
      typedef Internal::FFTWTraits<T> Traits;
      typedef typename Traits::Complex Complex;

      FFTWArray(size_t n)
        : n_(n)
        , a_(Traits::malloc(n))
        , norm_(n_) {}
      
      template<typename WINDOW_FCN>
      FFTWArray(const std::vector<std::complex<double> >& v,
                const WINDOW_FCN& window_fcn)
        : n_(v.size())
        , a_(Traits::malloc(n_))
        , norm_(fill(v, window_fcn)) {}
      
      ~FFTWArray() { Traits::free(a_); }
      
      size_t size() const { return n_; }
      T norm() const { return norm_; }
      
      Complex* begin() { return a_; }
      Complex* end()   { return a_+n_; }  
      Complex&       operator[](size_t n)       { return a_[n]; }
      const Complex& operator[](size_t n) const { return a_[n]; }
      
      template<typename V,
               template<typename U> class WINDOW_FCN>
      double fill(const std::vector<std::complex<V> >& v,
                  const WINDOW_FCN<V>& window_fcn) {
        return fill(v.begin(), v.end(), window_fcn);
      }
      template<typename V,
               template<typename U> class WINDOW_FCN>
      double fill(typename std::vector<std::complex<V> >::const_iterator i0,
                  typename std::vector<std::complex<V> >::const_iterator i1,
                  const WINDOW_FCN<V>& window_fcn) {
        const size_t length(std::distance(i0, i1));
        if (length != n_) resize(length);
        norm_= 0;
        for (unsigned u(0); u<n_ && i0 != i1; ++u, ++i0) {
          const T w(window_fcn(u,n_));
          norm_ += w;
          a_[u][0] = w * i0->real();
          a_[u][1] = w * i0->imag();
        }
        return norm_;
      }
      
      void resize(size_t n) {
        if (n != n_) {
          Traits::free(a_); 
          n_= n;
          a_= Traits::malloc(n);
        }
      }
    private:
      size_t n_;
      Complex *a_;
      T norm_;
    } ;
  } // namespace Internal

  template<typename T>
  class FFTWTransform : public Internal::FFTWTraits<T> {
  public:    
    typedef Internal::FFTWTraits<T> Traits;
    typedef typename Traits::Complex Complex;
    typedef typename Traits::Plan Plan;
    
    using Traits::plan_dft_1d;
    using Traits::execute;
    using Traits::destroy_plan;

    FFTWTransform(size_t n, int sign, unsigned flags)
      : in_(n)
      , out_(n)
      , sign_(sign)
      , flags_(flags)
      , plan_(plan_dft_1d(n, in_.begin(), out_.begin(), sign, flags))
      , normalizationFactor_(1.0/in_.norm()) {}    
    
    template<typename WINDOW_FCN>
    FFTWTransform(const std::vector<std::complex<double> >& v,
                  const WINDOW_FCN& window_fcn,
                  int sign, 
                  unsigned flags)
      : in_(v, window_fcn)
      , out_(v.size())
      , sign_(sign)
      , flags_(flags)
      , plan_(plan_dft_1d(in_.size(), in_.begin(), out_.begin(), sign, flags))
      , normalizationFactor_(T(1)/in_.norm()) { 
      execute(plan_);
    }    
    ~FFTWTransform() {
      destroy_plan(plan_);
    }

    void resize(size_t n) {
      if (n != size()) {
        in_.resize(n);
        out_.resize(n);
        destroy_plan(plan_);
        plan_ = plan_dft_1d(in_.size(), in_.begin(), out_.begin(), sign_, flags_);
        // normalizationFactor_ has to be updated by hand
      }
    }

    // norm of window
    double normWindow() const { return in_.size() * normalizationFactor_; }

    template<typename V,
             template <typename U> class WINDOW_FCN>
    void transformVector(const std::vector<std::complex<V> >& v,
                         const WINDOW_FCN<V>& window_fcn) {
      transformRange(v.begin(), v.end(), window_fcn);
    }
    template<typename V,
             template <typename U> class WINDOW_FCN>
    void transformRange(typename std::vector<std::complex<V> >::const_iterator i0,
                        typename std::vector<std::complex<V> >::const_iterator i1,
                        const WINDOW_FCN<V>& window_fcn) {
      const size_t length(std::distance(i0, i1));
      if (length != size())
        resize(length);
      in_.fill(i0, i1, window_fcn);
      normalizationFactor_= T(1)/in_.norm();
      execute(plan_);
    }

    size_t size() const { return in_.size(); }

    // corrected for spread due to window function
    std::complex<T> getBin(size_t u) const {
      return normalizationFactor_*std::complex<T>(out_[u][0], out_[u][1]); 
    }
    std::vector<Complex> getBins() const {
      std::vector<Complex> v(size());
      for (size_t u(0); u<size(); ++u) 
        v[u] = getBin(u);
      return v;
    }

    std::complex<T> getInBin(size_t u) const { 
      return std::complex<T>(in_[u][0], in_[u][1]);
    }

  protected:
  private:
    Internal::FFTWArray<T> in_;
    Internal::FFTWArray<T> out_;
    int sign_;
    unsigned flags_;
    Plan plan_;
    T normalizationFactor_;
  } ;
} // namespace FFT
#endif // _FFT_HPP_cm100823_
