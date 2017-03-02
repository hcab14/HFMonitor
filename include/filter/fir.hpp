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
#ifndef _FIR_HPP_cm110527_
#define _FIR_HPP_cm110527_

#include <algorithm>
#include <vector>
#include <complex>
#include <stdexcept>
#include "FFT.hpp"

namespace filter {
  namespace fir {
    namespace detail {
      template<typename T>
      class FFTWindowDesign {
      public:
        typedef T float_type;
        typedef typename std::complex<float_type> complex_type;
        typedef typename std::vector<float_type> real_vector_type;
        typedef typename std::vector<complex_type> complex_vector_type;

        FFTWindowDesign(size_t n)
          : b_(n,0) {
          if ((n % 2) == 0) throw std::runtime_error("FFTWindowDesign::FFTWindowDesign n is even");
        }
        virtual ~FFTWindowDesign() {}

        const real_vector_type& coeff() const { return b_; }

      protected:
        template<typename win_function_type>
        void design_(const complex_vector_type& f_pos,        // 0 and positive frequencies
                     const complex_vector_type& f_neg,        // 0 and negative frequencies
                     const win_function_type& win_function) { // window function
          if (f_pos.size() != f_neg.size())
            throw std::runtime_error("f_pos.size() != f_neg.size()");

          // arrange positive and negative frequencies
          complex_vector_type f(f_pos);
          std::reverse_copy(f_neg.begin()+1, f_neg.end()-1, std::back_inserter(f));
          const size_t m(f.size());

          //inverse FFT
          FFT::FFTWTransform<float_type> ifft(m, -1, FFTW_ESTIMATE);
          ifft.transformRange(f.begin(), f.end(), FFT::WindowFunction::Rectangular<float_type>(f.size()));

          // multiply with window function
          const size_t n(b_.size());
          float_t sum(0);
          float_t max_value(0);
          for (size_t i(0); i<n; ++i) {
            b_[i] = ifft.out((m+ i - n/2) % m).real() * win_function(i);
            sum += b_[i];
            max_value = (b_[i] > max_value) ? b_[i] : max_value;
          }

          // normalize
          const float_t norm(float_type(1)/sum);
          for (size_t i(0); i<n; ++i)
            b_[i] *= norm;
        }
      private:
        real_vector_type b_;
      } ;
    } // namespace detail

    template<typename T>
    class lowpass : public detail::FFTWindowDesign<T> {
    public:
      typedef detail::FFTWindowDesign<T> base_type;
      typedef typename base_type::float_type float_type;
      typedef typename base_type::complex_type complex_type;
      typedef typename base_type::real_vector_type real_vector_type;
      typedef typename base_type::complex_vector_type complex_vector_type;

      using base_type::coeff;
      using base_type::design_;

      lowpass(size_t n)
        : base_type(n) {
      }

      // f0 is normalized frequency (f0=f/fs)
      virtual void design(double f0, double f_ramp=0.) {
        if (f0     < 0.) throw std::runtime_error("lowpass::design f0 < 0");
        if (f_ramp < 0.) throw std::runtime_error("lowpass::design f_ramp < 0");
        const size_t n(coeff().size());
        const size_t m(compute_m(n));
        complex_vector_type v(m);
        const size_t if0(size_t(std::floor(f0*m)));
        for (size_t i(0); i<m; ++i)
          v[i] = (i<=if0);
        const size_t k(size_t(f_ramp*m));
//         std::cerr << "k= " << k << std::endl;
        for (size_t i(0); i<k; ++i) {
          const int j(int(if0+k/2)-int(i));
          if (j<0 || j>int(m)) continue;
          v[j] = double(i)/k;
        }

        base_type::design_(v, v, FFT::WindowFunction::Hamming<float_type>(v.size()));
      }

    protected:
    private:
      static size_t compute_m(size_t n) {
        size_t m(0);
        for (m=2; 2*m < n+1; m*=2) ;
        return m+1;
      }
      // real_vector_type b_;
    } ;
  } // namespace fir
} // namespace filter

#endif // _FIR_HPP_cm110527_
