#ifndef _CL_FFT_HPP_cm170305_
#define _CL_FFT_HPP_cm170305_

//#ifdef USE_OPENCL
#  define CL_USE_DEPRECATED_OPENCL_2_0_APIS
#  include <clFFT.h>
#  define __CL_ENABLE_EXCEPTIONS
#  include "cl/cl.hpp"
#  include "cl/error.hpp"
#  include "cl/array.hpp"
#  include "cl/setup.hpp"
//#endif

namespace cl {
  namespace fft {
    class setup {
    public:
      setup() {
	ASSERT_THROW_CL(clfftInitSetupData(&_setup_data));
	ASSERT_THROW_CL(clfftSetup        (&_setup_data));
      }
      ~setup() {
	clfftTeardown();
      }

    protected:
    private:
      clfftSetupData _setup_data;
    } ;

    class clfft {
    public:
      typedef boost::shared_ptr<clfft> sptr;
      typedef float value_type;
      typedef std::complex<float> complex_type;

      clfft(::size_t n,
	    clfftDirection direction,
	    cl::CommandQueue& q)
	: _in(q, 1024)
	, _out(q, 1024)
	, _queue(q)
	, _direction(direction)
	, _plan_handle()
	, _normalization_factor(1.0f) { resize(n); }

      virtual ~clfft() {
	clfftDestroyPlan(&_plan_handle);
      }

      ::size_t size() const { return _in.size(); }

      void resize(::size_t n) {
	if (n == _in.size() && n == _out.size())
	  return;

	_in.resize(n);
	_out.resize(n);

	::size_t clLengths[1] = { n };
	ASSERT_THROW_CL(clfftCreateDefaultPlan(&_plan_handle, _queue.getInfo<CL_QUEUE_CONTEXT>()(), CLFFT_1D, clLengths));
	ASSERT_THROW_CL(clfftSetPlanPrecision ( _plan_handle, CLFFT_SINGLE));
	ASSERT_THROW_CL(clfftSetPlanScale     ( _plan_handle, _direction, 1.0f));
	ASSERT_THROW_CL(clfftSetLayout        ( _plan_handle, CLFFT_COMPLEX_INTERLEAVED, CLFFT_COMPLEX_INTERLEAVED));
	ASSERT_THROW_CL(clfftSetResultLocation( _plan_handle, CLFFT_OUTOFPLACE));
	ASSERT_THROW_CL(clfftBakePlan         ( _plan_handle, 1, &_queue(), NULL, NULL));
      }

      void transform() {
	host_to_device();
	std::vector<cl::Event> events_wait;
	enqueue_transform(events_wait);
	_queue.finish();
	device_to_host();
      }
      template<typename IT,
	       typename W>
      void transformRange(IT i0,
			  IT i1,
			  const W& window_fcn) {
	const ::size_t length(std::distance(i0, i1));
	if (length != size())
	  resize(length);
	for (::size_t i=0; i<length; ++i)
	  in(i) = typename IT::value_type::value_type(window_fcn(i)) * *i0++;
	_normalization_factor = 1.0f/(float(length)*window_fcn.gain());
	transform();
      }

      cl::Event enqueue_transform(std::vector<cl::Event> &events_wait) {
	cl::Event event;
	ASSERT_THROW_CL(clfftEnqueueTransform(_plan_handle, _direction, 1, &_queue(),
					      events_wait.size(), events_wait.empty() ? NULL : &events_wait[0](), &event(),
					      &_in.get_device_buffer()(), &_out.get_device_buffer()(), NULL));
	return event;
      }

      void host_to_device(const std::vector<cl::Event>* events_wait=NULL, cl::Event* event_finished=NULL) {
	_in.host_to_device(_queue, events_wait, event_finished);
      }
      void device_to_host(const std::vector<cl::Event>* events_wait=NULL, cl::Event* event_finished=NULL) {
	_out.device_to_host(_queue, events_wait, event_finished);
      }

      complex_type& in (::size_t index) { return  _in[index]; }
      complex_type& out(::size_t index) { return _out[index]; }

      const complex_type& in (::size_t index) const { return  _in[index]; }
      const complex_type& out(::size_t index) const { return _out[index]; }

      array<complex_type>& in () { return  _in; }
      array<complex_type>& out() { return _out; }

      const array<complex_type>& in () const { return  _in; }
      const array<complex_type>& out() const { return _out; }

      complex_type getBin(::size_t i) const {
	return _normalization_factor*out(i);
      }
    protected:
    private:
      array<complex_type> _in;
      array<complex_type> _out;
      cl::CommandQueue    _queue;
      clfftDirection      _direction;
      clfftPlanHandle     _plan_handle;
      float               _normalization_factor;
    } ;

    class clfft_setup : public cl::setup {
    public:
      typedef clfft::sptr sptr;
      enum {
	FORWARD  = CLFFT_FORWARD,
	BACKWARD = CLFFT_BACKWARD
      };
      sptr make(::size_t n, int sign) {
	return boost::make_shared<sptr::element_type>(n, clfftDirection(sign), queue());
      }
    } ;

  } // namespace fft
} // namespace cl

#endif // _CL_FFT_HPP_cm170305_
