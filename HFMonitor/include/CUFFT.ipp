// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2013 Christoph Mayer
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
#ifndef _CUDA_FFT_IPP_cm140112_
#define _CUDA_FFT_IPP_cm140112_

#ifdef USE_CUDA
#include <cuda_runtime.h>
#include <cufft.h>
#include <helper_functions.h>
#include <helper_cuda.h>

namespace FFT {  
  namespace internal {
    // float precision only for now
    class CUFFTArray {
    public:
      typedef float2 Complex;

      // in order to avoid compiler warning (type-punned pointer ...) we use this
      union device_data {
        Complex* c; // as *Complex
        void*    v; // as *void
        void** as_vpp() { return reinterpret_cast<void **>(&v); }
      } ;

      CUFFTArray(size_t n)
        : host_vec_(n)
        , norm_(n) {
        checkCudaErrors(cudaMalloc(device_vec_.as_vpp(), n*sizeof(Complex)));
      }
      ~CUFFTArray() {
        cudaFree(device_vec_.c);
      }

      size_t size() const { return host_vec_.size(); }
      double norm() const { return norm_; }

      Complex&       operator[](size_t n)       { return host_vec_[n]; }
      const Complex& operator[](size_t n) const { return host_vec_[n]; }

      cufftComplex* device_vec() { return device_vec_.c; }

      void resize(size_t n) {
        host_vec_.resize(n);
        cudaFree(device_vec_.c);
        checkCudaErrors(cudaMalloc(device_vec_.as_vpp(), n*sizeof(Complex)));
      }

      void host_to_device() {
        checkCudaErrors(cudaMemcpy(device_vec_.c, &host_vec_.front(), size()*sizeof(Complex), cudaMemcpyHostToDevice));    
      }
      void device_to_host() {
        checkCudaErrors(cudaMemcpy(&host_vec_.front(), device_vec_.c, size()*sizeof(Complex), cudaMemcpyDeviceToHost));
      }
  
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
        if (length != size()) resize(length);
        norm_= 0;
        Complex cc;
        for (unsigned u(0), n(size()); u<n; ++u, ++i0) {
          const double w(window_fcn(u));
          norm_ += w;
#if 1
          cc.x = w * i0->real();
          cc.y = w * i0->imag();
          host_vec_[u] = cc;
#else 
          host_vec_[u].x = w * i0->real();
          host_vec_[u].y = w * i0->imag();
#endif
        }
        return norm_;
      }

    protected:
    private:
      std::vector<Complex> host_vec_;
      device_data          device_vec_;
      double               norm_;
    } ;
  } // namespace internal
  
  class CUFFTTransform {
  public:
    CUFFTTransform(size_t n, int direction, unsigned flags=0 /* not used, compatibility to FFTW*/)
      : vec_(n)
      , normalization_factor_(vec_.norm())
      , direction_(direction) {
      checkCudaErrors(cufftPlan1d(&plan_, n, CUFFT_C2C, 1));
      checkCudaErrors(cufftSetCompatibilityMode(plan_, CUFFT_COMPATIBILITY_FFTW_ALL));
    }
    ~CUFFTTransform() {
      cufftDestroy(plan_);
    }
    
    void resize(size_t n) {
      cufftDestroy(plan_);
      checkCudaErrors(cufftPlan1d(&plan_, n, CUFFT_C2C, 1));
      checkCudaErrors(cufftSetCompatibilityMode(plan_, CUFFT_COMPATIBILITY_FFTW_ALL));
      vec_.resize(n);
    }

    size_t size() const { return vec_.size(); }

    std::complex<float> getBin(size_t i) const {
      std::complex<float> r(vec_[i].x, vec_[i].y);
      r *= normalization_factor_;
      return r;
    }
  
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
      vec_.fill(i0, i1, window_fcn);
      normalization_factor_= 1./vec_.norm();
    
      host_to_device();
      transform();
      device_to_host();
    }
  
  protected:
    void host_to_device() {
      vec_.host_to_device();
    }
    void device_to_host() {
      vec_.device_to_host();
    }
    void transform() {
      checkCudaErrors(cufftExecC2C(plan_, vec_.device_vec(), vec_.device_vec(), direction_));
      checkCudaErrors(cudaThreadSynchronize());
    }
  private:
    cufftHandle          plan_;
    internal::CUFFTArray vec_;
    double               normalization_factor_;
    int                  direction_;
  } ;

} // namespace FFT

#endif // USE_CUDA
#endif // _CUDA_FFT_IPP_cm140112_
