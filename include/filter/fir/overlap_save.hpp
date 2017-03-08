// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2011 Christoph Mayer
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
#ifndef _OVERLAP_SAVE_HPP_cm110628_
#define _OVERLAP_SAVE_HPP_cm110628_

#include <iostream>
#include <map>

#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <volk/volk.h>

#include "aligned_vector.hpp"
#include "FFT.hpp"
#include "processor.hpp"
//
// multi overlap-save filter
//
namespace filter {
  namespace fir {
    class overlap_save {
    public:
      typedef boost::shared_ptr<overlap_save> sptr;
      typedef aligned_vector<float> real_vector_type;
      typedef FFT::FFTWTransform<float> small_fft_type;
#ifdef USE_CUDA
      typedef FFT::CUFFTTransform large_fft_type;
#else
      typedef FFT::FFTWTransform<float> large_fft_type;
#endif
      typedef processor::base_iq::complex_type complex_type;
      typedef processor::base_iq::vector_type complex_vector_type;
    private:

      // class for holding filter coefficients and the fft
      class filt {
      public:
        typedef boost::shared_ptr<filt> sptr;

        template<typename U>
        filt(size_t l,
             size_t p,
             const typename std::vector<U>& b,
             double shift=0., // normalized frequency
             size_t d=1)      // downsampling factor
          : l_(l)
          , p_(p)
          , d_(d)
          , n_(l_+p_-1)
          , shift_(size_t((1.+shift)*n_+0.5) % n_)
          , fft_ (n_,     1, FFTW_ESTIMATE)
          , ifft_(n_/d_, -1, FFTW_ESTIMATE)
          , h_(n_, 0)
          , result_(l_/d_, 0) {
          assert(l_+p_ > 0);
          assert(((p_-1)%d_) == 0);
          assert(b.size() < n());
          assert((n_%d_) == 0);
          assert((l_%d_) == 0);
          complex_vector_type in(n(), 0);
          std::copy(b.begin(), b.end(), in.begin());
          fft_.transformVector(in, FFT::WindowFunction::Rectangular<float>(b.size()));
          for (size_t i(0), iend(n()); i<iend; ++i) {
            h_[i] = fft_.out(i);
            fft_.in(i) = 0;
          }
          for (size_t i(0), nd(n()/d_); i<nd; ++i)
            ifft_.in(i) = 0;
          shift_ = (shift_/size_t(v()+.5))*size_t(v()+0.5);
        }
        ~filt() {}

        size_t l() const { return l_; }                  // Number of new input samples consumed per data block
        size_t p() const { return p_; }                  // Length of h(n)
        size_t d() const { return d_; }                  // downsampling factor
        size_t n() const { return n_; }                  // FFT size
        size_t shift() const { return shift_; }          // frequency shift
        double v() const { return double(n())/(p_-1.); } // Overlap factor

        double offset() const {
          return (shift() > n()/2) ? double(int(shift())-int(n()))/n(): double(shift())/n();
        }
	complex_vector_type::const_iterator begin()  const { return ifft_.out().begin()+(p()-1)/d(); }
        complex_vector_type::const_iterator end()    const { return ifft_.out().end(); }

        // performs inverse FFT of (shifted) input and downsampling
        void transform(const large_fft_type& fft) {
          const size_t nd(n()/d());
          const float norm(1.0f/float(n()));
#if 0
          // naive implementation
          for (size_t i(0); i<nd; ++i)
            ifft_.in(i) = 0;
          for (size_t i(0), iend(n()); i<iend; ++i) {
            ifft_.in(i%nd) += complex_multiplication_optimized
              (fft.out((n()+i-shift())%n()), h_[i]*norm);
          }
//#else
          // optimized loops
          const size_t m(n()/nd);
          for (size_t j(0); j<nd; ++j) {
            complex_type ci = 0;
            for (size_t i(0); i<m; ++i) {
              ci += complex_multiplication_optimized
                (fft.out((n()+j+i*nd-shift())%n()), norm*h_[j+i*nd]);
            }
            ifft_.in(j) = ci;
          }
#else
          // optimized loops
          const size_t m(n()/nd);
          aligned_vector<complex_type> x(m), a(m);
          for (size_t j(0); j<nd; ++j) {
            for (size_t i(0); i<m; ++i) {
              x[i] = fft.out((n()+j+i*nd-shift())%n());
              a[i] = h_[j+i*nd];
            }
            complex_type ci(0);
            volk_32fc_x2_dot_prod_32fc(&ci, &x[0], &a[0], m);
            ifft_.in(j) = norm*ci;
          }
#endif
          ifft_.transform();

          // for (size_t i(0), ld(l()/d()), offset((p()-1)/d()); i<ld; ++i)
          //   result_[i] = ifft_.out(offset+i);
        }

      protected:
      private:
        inline complex_type complex_multiplication_optimized(complex_type c1, complex_type c2) const {
          float a(c1.real()), b(c1.imag()),
            c(c2.real()), d(c2.imag());
          float k1(a*(c + d)),
            k2(d*(a + b)),
            k3(c*(b - a));
          return complex_type(k1-k2, k1+k3);
        }

        const size_t        l_;
        const size_t        p_;
        const size_t        d_;
        const size_t        n_;
        size_t              shift_;
        small_fft_type      fft_;    // fft
        small_fft_type      ifft_;   // fft^{-1}
        complex_vector_type h_;      // FFT of filter coefficients+padding
        complex_vector_type result_; // result
      } ;

    public:
      overlap_save(size_t l, // Number of new input samples consumed per data block
                   size_t p) // Length of h(n)
        : l_(l)
        , p_(p)
        , fft_(l+p-1, 1, FFTW_ESTIMATE)
        , last_id_(0) {
        for (size_t i(0), iend(l+p-1); i<iend; ++i)
          fft_.in(i) = 0;
      }

      virtual size_t l() const { return l_; }
      virtual size_t p() const { return p_; }

      typedef std::map<size_t, filt::sptr> filter_map;

      static size_t design_optimal(size_t p) {
        size_t n(1);
        while (n < p+1)
          n *= 2;
        double x     = complexity(n, p);
        double x_old = 2.*x+1.;
        while (x < x_old) {
          x_old = x;
          n    *= 2;
          x     = complexity(n, p);
        }
        return n/2;
      }
      static double complexity(size_t n, size_t p) {
        return (n+2.*n*std::log(n)/std::log(2))/(double(n)-double(p)+1.);
      }

      filt::sptr get_filter(size_t index) const {
        // return filters_[index];
	auto const& i = filters_.find(index);
	if (i != filters_.end())
	  return i->second;
	else
	  throw 1;
      }

      // add one filter
      //  * returns a pair of handle (size_t) and the (rounded) mid-frequency of the filter
      virtual std::pair<size_t, double> add_filter(const std::vector<float>& b,
                                                   double_t offset,
                                                   size_t decim) {
        if (b.size() != p_)
          throw std::runtime_error("overlap_save::update_filter_coeff b.size() != p_");
        filt::sptr fp(new filt(l_, p_, b, offset, decim));
        filters_.insert(std::make_pair(last_id_, fp));
        return std::make_pair(last_id_++, fp->offset());
      }

      void proc(const complex_vector_type& in) {
        proc(in.begin(), in.end());
      }

      virtual complex_vector_type::const_iterator begin(::size_t idx) const {
	return get_filter(idx)->begin();
      }
      virtual complex_vector_type::const_iterator end  (::size_t idx) const {
	return get_filter(idx)->end();
      }

      void proc(typename complex_vector_type::const_iterator i0,
                typename complex_vector_type::const_iterator i1) {
        if (std::distance(i0, i1) != int(l_))
          throw std::runtime_error(str(boost::format("overlap_save::proc in.size() != l_ : %d != %d")
                                       % std::distance(i0, i1)
                                       % int(l_)));

        // copy input data
        for (size_t i(0); i<l_; ++i)
          fft_.in(p_+i-1) = *i0++;
        fft_.transform();

        // for each filter
        for (auto filter : filters_)
          filter.second->transform(fft_);

        // save old samples
        for (size_t i(0), iend(p_-1); i<iend; ++i)
          fft_.in(i) = fft_.in(l_+i);
      }

    private:
      const size_t l_;
      const size_t p_;
      large_fft_type fft_;
      size_t last_id_;
      filter_map filters_;
    } ;

    struct overlap_save_setup {
      typedef overlap_save::sptr sptr;
      sptr make_overlap_save(::size_t l, ::size_t p) {
	return boost::make_shared<overlap_save>(l, p);
      }
    } ;

  } // namespace fir
} // namespace filter
#endif // _OVERLAP_SAVE_HPP_cm110628_
