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

#include <boost/shared_ptr.hpp>

#include "FFT.hpp"

// multi overlap-save filter
// 
namespace filter {
  namespace fir {
    template<typename T>
    class overlap_save {
    public:  
      typedef std::complex<T> complex_type;
      typedef std::vector<T> real_vector_type;
      typedef std::vector<complex_type> complex_vector_type;
      typedef FFT::FFTWTransform<T> fft_type;
    private:

      // class for holding filter coefficients and the fft
      class filt {
      public:
        typedef typename boost::shared_ptr<filt> sptr;

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
          fft_.transformVector(in, FFT::WindowFunction::Rectangular<T>());
          for (size_t i(0); i<n(); ++i) {
            h_[i] = fft_.out(i);
            fft_.in(i) = 0;
          }
          for (size_t i(0); i<n()/d_; ++i)
            ifft_.in(i) = 0;
          shift_ = (shift_/size_t(v()))*size_t(v());
          std::cout << "v= " << v() << std::endl;
        }
        ~filt() {}

        size_t l() const { return l_; }                  // Number of new input samples consumed per data block
        size_t p() const { return p_; }                  // Length of h(n)
        size_t d() const { return d_; }                  // downsampling factor
        size_t n() const { return n_; }                  // FFT size
        size_t shift() const { return shift_; }          // frequency shift
        double v() const { return double(n())/(p_-1.); } // Overlap factor

        double offset() const {
          return (shift() > n()/2) ? double(int(shift())-n())/n(): double(shift())/n();
        }
        typename complex_vector_type::const_iterator begin() const {
          return result_.begin();
        }
        typename complex_vector_type::const_iterator end() const {
          return result_.end();
        }
        const complex_vector_type& result() const { return result_; }

        // performs inverse FFT of (shifted) input and downsampling       
        void transform(const fft_type& fft) {
          std::cout << "#shift " << shift() << " l=" << l()<< std::endl;
          for (size_t i(0); i<n()/d(); ++i)
            ifft_.in(i) = 0;
          for (size_t i(0); i<n(); ++i)
            ifft_.in(i%(n()/d())) += fft.out((n()+i-shift())%n())*h_[i]/T(l_);
          ifft_.transform();
          
          for (size_t i(0); i<l()/d(); ++i)
            result_[i] = ifft_.out((p()-1)/d()+i);
        }

      protected:
      private:
        const size_t        l_;
        const size_t        p_;
        const size_t        d_;
        const size_t        n_;
        size_t              shift_;
        fft_type            fft_;    // fft
        fft_type            ifft_;   // fft^{-1}
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
        for (size_t i(0); i<l+p-1; ++i)
          fft_.in(i) = 0;
      }

      typedef std::map<size_t, typename filt::sptr> filter_map;

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
      
      typename filt::sptr get_filter(size_t index) {
        return filters_[index];
      }

      // add one filter
      //  * returns a pair of handle (size_t) and the mid-frequency of the filter
      template<typename U>
      std::pair<size_t, double> add_filter(const typename std::vector<U>& b,
                                           double_t offset,
                                           size_t decim) {
        if (b.size() != p_)
          throw std::runtime_error("overlap_save::update_filter_coeff b.size() != p_");
        typename filt::sptr fp(new filt(l_, p_, b, offset, decim));
        filters_.insert(std::make_pair(last_id_, fp));
        return std::make_pair(last_id_++, fp->offset());
      }
      
      void proc(const complex_vector_type& in) {
        if (in.size() != l_)
          throw std::runtime_error("overlap_save::proc in.size() != l_");

        // copy input data
        for (size_t i(0); i<l_; ++i)
          fft_.in(p_+i-1) = in[i];
        fft_.transform();

        // for each filter
        for (typename filter_map::iterator i(filters_.begin()); i != filters_.end(); ++i) {
          typename filt::sptr fp(i->second);
          fp->transform(fft_);
        }

        // save old samples
        for (size_t i=0; i<p_-1; ++i)
          fft_.in(i) = fft_.in(l_+i);

      }
      
    private:
      const size_t l_;
      const size_t p_;
      fft_type fft_;
      size_t last_id_;
      filter_map filters_;
    } ;
  } // namespace fir
} // namespace filter
#endif // _OVERLAP_SAVE_HPP_cm110628_
