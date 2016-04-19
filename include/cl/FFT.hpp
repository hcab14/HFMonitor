#ifndef _CL_FFT_HPP_cm160419_
#define _CL_FFT_HPP_cm160419_

#include <complex>
#include <vector>

#include "cl/cl.hpp"
#include "clFFT.h"

#include "FFT.hpp"
#include "logging.hpp"

namespace CL {
  namespace FFT {
    namespace internal {
      class Array {
      public:
	typedef std::complex<float> complex_type;
	
	Array(const CL::context& ctx,
	      size_t n)
	  : _v(n,complex_type(0))
	  , _norm(n) {
	  cl_int err(CL_SUCCESS);
	  _buf = clCreateBuffer(ctx, CL_MEM_READ_WRITE, n*sizeof(complex_type), NULL, &err);
	  ASSERT_THROW_CL(err);
	}
	~Array() {
	  ASSERT_THROW_CL(clReleaseMemObject(_buf));
	}

	void resize(size_t n) {
	  if (_v.size() == n)
	    return;
	  clReleaseMemObject(_buf);
	  _v.resize(n, complex_type(0));
	  cl_int err(CL_SUCCESS);
	  _buf = clCreateBuffer(context(), CL_MEM_READ_WRITE, n*sizeof(complex_type), NULL, &err);
	  ASSERT_THROW_CL(err);
	  _norm = n;
	}

	CL::context context() const {
	  cl_context ctx;
	  size_t n(0);
	  ASSERT_THROW_CL(clGetMemObjectInfo(_buf, CL_MEM_CONTEXT, sizeof(ctx), &ctx, &n));
	  ASSERT_THROW(n == sizeof(ctx));
	  return CL::context(ctx);
	}
	
	operator cl_mem() { return _buf; }
	
	size_t size() const { return _v.size(); }
	float norm() const { return _norm; }
	
	complex_type* begin() { return &_v[0]; }
	complex_type* end()   { return &_v[0]+_v.size(); }  
	
	complex_type& operator[](size_t i) { return _v[i]; }
	const complex_type& operator[](size_t i) const { return _v[i]; }
	
	void host_to_device(const CL::queue& q) {
	  ASSERT_THROW_CL(clEnqueueWriteBuffer(q, _buf, CL_TRUE, 0, _v.size()*sizeof(complex_type), &_v[0], 0, NULL, NULL));
	  ASSERT_THROW_CL(clFinish(q));
	}
	void device_to_host(const CL::queue& q) {
	  ASSERT_THROW_CL(clEnqueueReadBuffer(q, _buf, CL_TRUE, 0, _v.size()*sizeof(complex_type), &_v[0], 0, NULL, NULL ));
	  ASSERT_THROW_CL(clFinish(q));
	}
	
	template<typename V, typename W>
	float fill(const V& v,
		   const W& window_fcn) {
	  return fill(v.begin(), v.end(), window_fcn);
	}

	template<typename IT, typename W>
	float fill(IT i0, IT i1, W window_fcn) {
	  const ssize_t length(std::distance(i0, i1));
	  if (length != _v.size())
	    resize(length);
	  _norm= 0;
	  for (unsigned u(0), n(_v.size()); u<n; ++u, ++i0) {
	    const float w(window_fcn(u));
	    _norm += w;
	    _v[u]  = w* (*i0);
	  }
	  return _norm;
	}
	
      protected:
      private:
	cl_mem _buf;
	std::vector<complex_type> _v;
	float _norm;
      } ;
      
    } // namespace internal
    
    class setup {
    public:
      setup() {
	ASSERT_THROW_CL(clfftInitSetupData(&_fftSetup));
	ASSERT_THROW_CL(clfftSetup(&_fftSetup));
      }
      ~setup() {
	ASSERT_THROW_CL(clfftTeardown());
      }
    protected:
    private:
      clfftSetupData  _fftSetup;
    } ;
    
    class clFFT {
    public:
      clFFT(const CL::context& ctx,
	    const CL::queue& q,
	    size_t n,	    
	    clfftDirection dir=CLFFT_FORWARD)
	: _in(ctx, n)
	, _out(ctx, n)
	, _dir(dir) {
	resize(ctx, q, n, false);
      }
      
      ~clFFT() {
	ASSERT_THROW_CL(clfftDestroyPlan(&_planHandle));
      }
      
      void resize(const CL::context& ctx,
		  const CL::queue& q,
		  size_t n,
		  bool destroyPlan=true) {
	const clfftDim dim = CLFFT_1D;
	size_t clLengths[1] = {n};
      
	if (destroyPlan)
	  ASSERT_THROW_CL(clfftDestroyPlan(&_planHandle));
	ASSERT_THROW_CL(clfftCreateDefaultPlan(&_planHandle, ctx, dim, clLengths));
	
	ASSERT_THROW_CL(clfftSetPlanPrecision (_planHandle, CLFFT_SINGLE));
	ASSERT_THROW_CL(clfftSetLayout        (_planHandle, CLFFT_COMPLEX_INTERLEAVED, CLFFT_COMPLEX_INTERLEAVED));
	ASSERT_THROW_CL(clfftSetResultLocation(_planHandle, CLFFT_OUTOFPLACE));
	
	cl_command_queue qs[1] = { q };
	ASSERT_THROW_CL(clfftBakePlan(_planHandle, 1, qs, NULL, NULL));
      }
      
      internal::Array::complex_type& in(size_t i) { return _in[i]; }
      const internal::Array::complex_type& in(size_t i) const { return _in[i]; }

      internal::Array::complex_type& out(size_t i) { return _out[i]; }
      const internal::Array::complex_type& out(size_t i) const { return _out[i]; }
      
      void transform(const CL::queue& q) {
	_in.host_to_device(q);
	transform_on_device(q);
	_out.device_to_host(q);
      }
      
    protected:
      void transform_on_device(const CL::queue& q) {
	cl_command_queue qs[1]     = { q    };
	cl_mem           ms_in[1]  = { _in  };
	cl_mem           ms_out[1] = { _out };
	ASSERT_THROW_CL(clfftEnqueueTransform(_planHandle, _dir, 1, qs, 0, NULL, NULL, ms_in, ms_out, NULL));
	ASSERT_THROW_CL(clFinish(q));
      }
      
    private:
      clfftPlanHandle _planHandle;
      internal::Array _in;
      internal::Array _out;
      clfftDirection  _dir;
    } ;

  } //namespace FFT  
} // namespace CL

#endif // _CL_FFT_HPP_cm160419_
