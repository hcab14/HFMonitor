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
    class array {
    public:
      typedef std::complex<float> complex_type;
	
      array(const CL::context& ctx,
	    size_t n)
	: _mem(ctx, sizeof(complex_type)*n)
	, _v(n,complex_type(0))
	, _norm(n) {}

      array(const array& a) 
	: _mem(a._mem)
	, _v(a._v)
	, _norm(a._norm) {
	std::cout << "array::array(const array&) " << this << std::endl;
      }

      ~array() {
	std::cout << "~array " << this << " "<< cl_mem(_mem) << std::endl;
      }

      operator cl_mem() const {
	return _mem;
      }

      CL::context get_context() const { return _mem.get_context(); }

      void resize(size_t n) {
	if (_v.size()*sizeof(complex_type) == n)
	  return;
	_v.resize(n, complex_type(0));
	_mem = CL::mem(_mem.get_context(), sizeof(complex_type)*n);
	_norm = n;
      }

      size_t size() const { return _v.size(); }
      float norm() const { return _norm; }
	
      complex_type* begin() { return &_v[0]; }
      complex_type* end()   { return &_v[0]+_v.size(); }  
	
      complex_type& operator[](size_t i) { return _v[i]; }
      const complex_type& operator[](size_t i) const { return _v[i]; }
	
      void host_to_device(CL::queue& q) {
	q.enqueueWriteBuffer(_mem, _v.size()*sizeof(complex_type), &_v[0]);
      }
      void device_to_host(CL::queue& q) {
	q.enqueueReadBuffer(_mem, _v.size()*sizeof(complex_type), &_v[0]);
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
      CL::mem _mem;
      std::vector<complex_type> _v;
      float _norm;
    } ;
      
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
      clFFT(CL::queue& q,
	    const array& in,
	    const array& out,
	    clfftDirection dir=CLFFT_FORWARD)
	: _in(in)
	, _out(out)
	, _dir(dir) {
	ASSERT_THROW(_in.size() == _out.size());
	resize(_in.get_context(), q, _in.size(), false);
      }
      clFFT(const CL::context& ctx,
	    CL::queue& q,
	    size_t n,	    
	    clfftDirection dir=CLFFT_FORWARD)
	: _in(ctx, n)
	, _out(ctx, n)
	, _dir(dir) {
	resize(ctx, q, n, false);
      }
      
      ~clFFT() {
	std::cout << "A0" << std::endl;
	ASSERT_THROW_CL(clfftDestroyPlan(&_planHandle));
	std::cout << "A1" << std::endl;
      }
      
      void resize(const CL::context& ctx,
		  CL::queue& q,
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
      
      array::complex_type& in(size_t i) { return _in[i]; }
      const array::complex_type& in(size_t i) const { return _in[i]; }

      array::complex_type& out(size_t i) { return _out[i]; }
      const array::complex_type& out(size_t i) const { return _out[i]; }
      
      void transform(CL::queue& q) {
	_in.host_to_device(q);
	transform_on_device(q);
	_out.device_to_host(q);
      }
      
    protected:
      CL::event transform_on_device(CL::queue& q) {
	const std::vector<CL::event> wait_list;
	return transform_on_device(q, wait_list);
      }
      CL::event transform_on_device(CL::queue& q, const std::vector<CL::event>& wait_list) {
	const std::vector<cl_event> event_list(CL::util::make_vector(wait_list));
	cl_command_queue qs[1]     = { q    };
	cl_mem           ms_in[1]  = { _in  };
	cl_mem           ms_out[1] = { _out };
	cl_event         es[1];
	ASSERT_THROW_CL(clfftEnqueueTransform(_planHandle, _dir, 1, qs, event_list.size(), &event_list[0], es, ms_in, ms_out, NULL));
	return CL::event(es[0], false);
      }
      
    private:
      clfftPlanHandle _planHandle;
      array _in;
      array _out;
      clfftDirection  _dir;
    } ;

  } //namespace FFT  
} // namespace CL

#endif // _CL_FFT_HPP_cm160419_
