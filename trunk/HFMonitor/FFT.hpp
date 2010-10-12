// -*- C++ -*-
// $Id$
#ifndef _FFT_HPP_cm230810_
#define _FFT_HPP_cm230810_

#include <fftw3.h>

namespace FFT {  
  namespace WindowFunction {
    struct Rectangular {
      double operator()(size_t u, size_t n) const { return 1.; }
    } ;
    struct Hanning {
      double operator()(size_t u, size_t n) const { 
	return 0.5*(1.0-std::cos(2.*M_PI*u/double(n-1)));
      }
    } ;
    struct Hamming {
      double operator()(size_t u, size_t n) const { 
	return 0.54 - 0.46*std::cos(2.*M_PI*u/double(n-1));
      }
    } ;
    class Gaussian {
    public:
      Gaussian(double sigma)
	: sigma_(sigma) {}
      double operator()(size_t u, size_t n) const { 
	return std::exp(-0.5*std::pow((u-0.5*(n-1)) / (0.5*sigma_*(n-1)), 2));
      }
    private:
      const double sigma_;
    } ;
    class Blackman {
    public:
      Blackman(double alpha=0.16)
	: a0_(0.5*(1-alpha))
	, a1_(0.5)
	, a2_(0.5*alpha) {}
      double operator()(size_t u, size_t n) const { 
	const double t(2.*M_PI*u/(n-1));
	return a0_ - a1_*std::cos(t) + a2_*std::cos(2*t);
      }
    private:
      const double a0_, a1_, a2_;
    } ;
  } // namespace WindowFunction

  namespace Internal {
    class FFTWArray {
    public:
      FFTWArray(size_t n)
	: n_(n)
	, a_ ((fftw_complex*)fftw_malloc(sizeof(fftw_complex) * n))
	, norm_(n_) {}
      
      template<typename WINDOW_FCN>
      FFTWArray(const std::vector<std::complex<double> >& v,
		const WINDOW_FCN& window_fcn)
	: n_(v.size())
	, a_((fftw_complex*)fftw_malloc(sizeof(fftw_complex) * n_))
	, norm_(fill(v, window_fcn)) {}
      
      ~FFTWArray() { fftw_free(a_); }
      
      size_t size() const { return n_; }
      double norm() const { return norm_; }
      
      fftw_complex* begin() { return a_; }
      fftw_complex* end()   { return a_+n_; }  
      fftw_complex&       operator[](size_t n)       { return a_[n]; }
      const fftw_complex& operator[](size_t n) const { return a_[n]; }
      
      template<typename WINDOW_FCN>
      double fill(const std::vector<std::complex<double> >& v,
		  const WINDOW_FCN& window_fcn) {
	return fill(v.begin(), v.end(), window_fcn);
      }
      template<typename WINDOW_FCN>
      double fill(std::vector<std::complex<double> >::const_iterator i0,
		  std::vector<std::complex<double> >::const_iterator i1,
		  const WINDOW_FCN& window_fcn) {
	const size_t length(std::distance(i0, i1));
	if (length != n_) resize(length);
	norm_= 0;
	for (unsigned u(0); u<n_ && i0 != i1; ++u, ++i0) {
	  const double w(window_fcn(u,n_));
	  norm_ += w;
	  a_[u][0] = w * i0->real();
	  a_[u][1] = w * i0->imag();
	}
	return norm_;
      }
      
      void resize(size_t n) {
	if (n != n_) {
	  fftw_free(a_); 
	  n_= n;
	  a_= (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * n_);
	}
      }
    private:
      size_t n_;
      fftw_complex *a_;
      double norm_;
    } ;
  } // namespace Internal

  class FFTWTransform {
  public:    
    FFTWTransform(size_t n, int sign, unsigned flags)
      : in_(n)
      , out_(n)
      , sign_(sign)
      , flags_(flags)
      , plan_(fftw_plan_dft_1d(n, in_.begin(), out_.begin(), sign, flags))
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
      , plan_(fftw_plan_dft_1d(in_.size(), in_.begin(), out_.begin(), sign, flags))
      , normalizationFactor_(1.0/in_.norm()) { 
      fftw_execute(plan_);
    }    
    ~FFTWTransform() {
      fftw_destroy_plan(plan_);
    }

    void resize(size_t n) {
      if (n != size()) {
	std::cout << "#FFTW resize " << n << std::endl;
	in_.resize(n);
	out_.resize(n);
	fftw_destroy_plan(plan_);
	plan_ = fftw_plan_dft_1d(in_.size(), in_.begin(), out_.begin(), sign_, flags_);
      }
    }

    template<typename WINDOW_FCN>
    void transformVector(const std::vector<std::complex<double> >& v,
			 const WINDOW_FCN& window_fcn) {
      transformRange(v.begin(), v.end(), window_fcn);
    }
    template<typename WINDOW_FCN>
    void transformRange(std::vector<std::complex<double> >::const_iterator i0,
			std::vector<std::complex<double> >::const_iterator i1,
			const WINDOW_FCN& window_fcn) {
      const size_t length(std::distance(i0, i1));
      if (length != size())
	resize(length);
      in_.fill(i0, i1, window_fcn);
      normalizationFactor_= 1.0/in_.norm();
      fftw_execute(plan_);
    }

    size_t size() const { return in_.size(); }
    std::complex<double> getBin(size_t u) const { 
      return normalizationFactor_*std::complex<double>(out_[u][0], out_[u][1]); 
    }

    std::complex<double> getInBin(size_t u) const { 
      return std::complex<double>(in_[u][0], in_[u][1]);
    }

    std::vector<std::complex<double> > getBins() const {
      std::vector<std::complex<double> > v(size());
      for (size_t u(0); u<size(); ++u) 
	v[u] = getBin(u);
      return v;
    }

  protected:
  private:
    Internal::FFTWArray in_;
    Internal::FFTWArray out_;
    int       sign_;
    unsigned  flags_;
    fftw_plan plan_;
    double    normalizationFactor_;
  } ;
} // namespace FFT
#endif // _FFT_HPP_cm230810_
