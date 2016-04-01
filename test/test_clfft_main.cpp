#include <iostream>
#include <complex>
#include <vector>
#include <unistd.h>

#include <boost/current_function.hpp>
#include <boost/format.hpp>
#include <clFFT.h>

#include "FFT.hpp"
#include "logging.hpp"

namespace CL {  
  struct Error {
    static const char* toString(cl_int err) {
      switch (err) {
      case CL_SUCCESS                                   : return "CL_SUCCESS";
      case CL_DEVICE_NOT_FOUND                          : return "CL_DEVICE_NOT_FOUND";
      case CL_DEVICE_NOT_AVAILABLE                      : return "CL_DEVICE_NOT_AVAILABLE";
      case CL_COMPILER_NOT_AVAILABLE                    : return "CL_COMPILER_NOT_AVAILABLE";
      case CL_MEM_OBJECT_ALLOCATION_FAILURE             : return "CL_MEM_OBJECT_ALLOCATION_FAILURE";
      case CL_OUT_OF_RESOURCES                          : return "CL_OUT_OF_RESOURCES";
      case CL_OUT_OF_HOST_MEMORY                        : return "CL_OUT_OF_HOST_MEMORY";
      case CL_PROFILING_INFO_NOT_AVAILABLE              : return "CL_PROFILING_INFO_NOT_AVAILABLE";
      case CL_MEM_COPY_OVERLAP                          : return "CL_MEM_COPY_OVERLAP";
      case CL_IMAGE_FORMAT_MISMATCH                     : return "CL_IMAGE_FORMAT_MISMATCH";
      case CL_IMAGE_FORMAT_NOT_SUPPORTED                : return "CL_IMAGE_FORMAT_NOT_SUPPORTED";
      case CL_BUILD_PROGRAM_FAILURE                     : return "CL_BUILD_PROGRAM_FAILURE";
      case CL_MAP_FAILURE                               : return "CL_MAP_FAILURE";
      case CL_MISALIGNED_SUB_BUFFER_OFFSET              : return "CL_MISALIGNED_SUB_BUFFER_OFFSET";
      case CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST : return "CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST";
#ifdef CL_VERSION_1_2
      case CL_COMPILE_PROGRAM_FAILURE                   : return "CL_COMPILE_PROGRAM_FAILURE";
      case CL_LINKER_NOT_AVAILABLE                      : return "CL_LINKER_NOT_AVAILABLE";
      case CL_LINK_PROGRAM_FAILURE                      : return "CL_LINK_PROGRAM_FAILURE";
      case CL_DEVICE_PARTITION_FAILED                   : return "CL_DEVICE_PARTITION_FAILED";
      case CL_KERNEL_ARG_INFO_NOT_AVAILABLE             : return "CL_KERNEL_ARG_INFO_NOT_AVAILABLE";
#endif
      case CL_INVALID_VALUE                             : return "CL_INVALID_VALUE";
      case CL_INVALID_DEVICE_TYPE                       : return "CL_INVALID_DEVICE_TYPE";
      case CL_INVALID_PLATFORM                          : return "CL_INVALID_PLATFORM";
      case CL_INVALID_DEVICE                            : return "CL_INVALID_DEVICE";
      case CL_INVALID_CONTEXT                           : return "CL_INVALID_CONTEXT";
      case CL_INVALID_QUEUE_PROPERTIES                  : return "CL_INVALID_QUEUE_PROPERTIES";
      case CL_INVALID_COMMAND_QUEUE                     : return "CL_INVALID_COMMAND_QUEUE";
      case CL_INVALID_HOST_PTR                          : return "CL_INVALID_HOST_PTR";
      case CL_INVALID_MEM_OBJECT                        : return "CL_INVALID_MEM_OBJECT";
      case CL_INVALID_IMAGE_FORMAT_DESCRIPTOR           : return "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR";
      case CL_INVALID_IMAGE_SIZE                        : return "CL_INVALID_IMAGE_SIZE";
      case CL_INVALID_SAMPLER                           : return "CL_INVALID_SAMPLER";
      case CL_INVALID_BINARY                            : return "CL_INVALID_BINARY";
      case CL_INVALID_BUILD_OPTIONS                     : return "CL_INVALID_BUILD_OPTIONS";
      case CL_INVALID_PROGRAM                           : return "CL_INVALID_PROGRAM";
      case CL_INVALID_PROGRAM_EXECUTABLE                : return "CL_INVALID_PROGRAM_EXECUTABLE";
      case CL_INVALID_KERNEL_NAME                       : return "CL_INVALID_KERNEL_NAME";
      case CL_INVALID_KERNEL_DEFINITION                 : return "CL_INVALID_KERNEL_DEFINITION";
      case CL_INVALID_KERNEL                            : return "CL_INVALID_KERNEL";
      case CL_INVALID_ARG_INDEX                         : return "CL_INVALID_ARG_INDEX";
      case CL_INVALID_ARG_VALUE                         : return "CL_INVALID_ARG_VALUE";
      case CL_INVALID_ARG_SIZE                          : return "CL_INVALID_ARG_SIZE";
      case CL_INVALID_KERNEL_ARGS                       : return "CL_INVALID_KERNEL_ARGS";
      case CL_INVALID_WORK_DIMENSION                    : return "CL_INVALID_WORK_DIMENSION";
      case CL_INVALID_WORK_GROUP_SIZE                   : return "CL_INVALID_WORK_GROUP_SIZE";
      case CL_INVALID_WORK_ITEM_SIZE                    : return "CL_INVALID_WORK_ITEM_SIZE";
      case CL_INVALID_GLOBAL_OFFSET                     : return "CL_INVALID_GLOBAL_OFFSET";
      case CL_INVALID_EVENT_WAIT_LIST                   : return "CL_INVALID_EVENT_WAIT_LIST";
      case CL_INVALID_EVENT                             : return "CL_INVALID_EVENT";
      case CL_INVALID_OPERATION                         : return "CL_INVALID_OPERATION";
      case CL_INVALID_GL_OBJECT                         : return "CL_INVALID_GL_OBJECT";
      case CL_INVALID_BUFFER_SIZE                       : return "CL_INVALID_BUFFER_SIZE";
      case CL_INVALID_MIP_LEVEL                         : return "CL_INVALID_MIP_LEVEL";
      case CL_INVALID_GLOBAL_WORK_SIZE                  : return "CL_INVALID_GLOBAL_WORK_SIZE";
      case CL_INVALID_PROPERTY                          : return "CL_INVALID_PROPERTY";
#ifdef CL_VERSION_1_2
      case CL_INVALID_IMAGE_DESCRIPTOR                  : return "CL_INVALID_IMAGE_DESCRIPTOR";
      case CL_INVALID_COMPILER_OPTIONS                  : return "CL_INVALID_COMPILER_OPTIONS";
      case CL_INVALID_LINKER_OPTIONS                    : return "CL_INVALID_LINKER_OPTIONS";
      case CL_INVALID_DEVICE_PARTITION_COUNT            : return "CL_INVALID_DEVICE_PARTITION_COUNT";
#endif
#ifdef cl_khr_icd
      case CL_PLATFORM_NOT_FOUND_KHR                    : return "CL_PLATFORM_NOT_FOUND_KHR";
#endif
#ifdef cl_khr_d3d10_sharing
      case CL_INVALID_D3D10_DEVICE_KHR                  : return "CL_INVALID_D3D10_DEVICE_KHR";
      case CL_INVALID_D3D10_RESOURCE_KHR                : return "CL_INVALID_D3D10_RESOURCE_KHR";
      case CL_D3D10_RESOURCE_ALREADY_ACQUIRED_KHR       : return "CL_D3D10_RESOURCE_ALREADY_ACQUIRED_KHR";
      case CL_D3D10_RESOURCE_NOT_ACQUIRED_KHR           : return "CL_D3D10_RESOURCE_NOT_ACQUIRED_KHR";
#endif
#ifdef cl_khr_d3d11_sharing
      case CL_INVALID_D3D11_DEVICE_KHR                  : return "CL_INVALID_D3D11_DEVICE_KHR";
      case CL_INVALID_D3D11_RESOURCE_KHR                : return "CL_INVALID_D3D11_RESOURCE_KHR";
      case CL_D3D11_RESOURCE_ALREADY_ACQUIRED_KHR       : return "CL_D3D11_RESOURCE_ALREADY_ACQUIRED_KHR";
      case CL_D3D11_RESOURCE_NOT_ACQUIRED_KHR           : return "CL_D3D11_RESOURCE_NOT_ACQUIRED_KHR";
#endif
#ifdef cl_khr_dx9_media_sharing
      case CL_INVALID_DX9_MEDIA_ADAPTER_KHR             : return "CL_INVALID_DX9_MEDIA_ADAPTER_KHR";
      case CL_INVALID_DX9_MEDIA_SURFACE_KHR             : return "CL_INVALID_DX9_MEDIA_SURFACE_KHR";
      case CL_DX9_MEDIA_SURFACE_ALREADY_ACQUIRED_KHR    : return "CL_DX9_MEDIA_SURFACE_ALREADY_ACQUIRED_KHR";
      case CL_DX9_MEDIA_SURFACE_NOT_ACQUIRED_KHR        : return "CL_DX9_MEDIA_SURFACE_NOT_ACQUIRED_KHR";
#endif

#ifdef clfftVersionMajor
      case CLFFT_BUGCHECK                  : return "CLFFT_BUGCHECK";
      case CLFFT_NOTIMPLEMENTED            : return "CLFFT_NOTIMPLEMENTED";
      case CLFFT_TRANSPOSED_NOTIMPLEMENTED : return "CLFFT_TRANSPOSED_NOTIMPLEMENTED";
      case CLFFT_FILE_NOT_FOUND            : return "CLFFT_FILE_NOT_FOUND";
      case CLFFT_FILE_CREATE_FAILURE       : return "CLFFT_FILE_CREATE_FAILURE";
      case CLFFT_VERSION_MISMATCH          : return "CLFFT_VERSION_MISMATCH";
      case CLFFT_INVALID_PLAN              : return "CLFFT_INVALID_PLAN";
      case CLFFT_DEVICE_NO_DOUBLE          : return "CLFFT_DEVICE_NO_DOUBLE";
      case CLFFT_DEVICE_MISMATCH           : return "CLFFT_DEVICE_MISMATCH";
#endif
      default: return "unknown CL error";
      }
    }
  } ;
  
} // namespace CL

#ifndef THROW_SITE_INFO
#define THROW_SITE_INFO(what)                                           \
  std::string(std::string(what) + "\n" +                                \
	      "  in " + std::string(BOOST_CURRENT_FUNCTION) + "\n" +    \
	      "  at " + std::string(__FILE__) + ":" +                   \
	      BOOST_STRINGIZE(__LINE__) + "\n")
#endif
#define ASSERT_THROW_CL(_x) {						\
    const cl_int __cl_error_code__((_x));				\
    if (__cl_error_code__ != CL_SUCCESS)  {				\
      throw std::runtime_error(THROW_SITE_INFO("assertion failed: "     \
					       + std::string(#_x) + " '" \
					       + std::string(CL::Error::toString(__cl_error_code__))+"'")); \
    } else void(0); }

namespace CL {
  class device {
  public:

    device(cl_device_id id=0)
      : _id(id) {}
    
    cl_device_id id() const { return _id; }

    std::string name() const {
      char device_name[128];
      size_t n(0);
      ASSERT_THROW_CL(clGetDeviceInfo(_id, CL_DEVICE_NAME, sizeof(device_name), device_name, &n));
      return std::string(device_name, n-1);
    }

  protected:
  private:
    cl_device_id _id;
  } ;

  class platform {
  public:
    platform(cl_platform_id id=0)
      : _id(id) {}

    cl_platform_id id() const { return _id; }

    std::string info(cl_platform_info info=CL_PLATFORM_NAME) const {
      char name[1024];
      size_t ret_param_size = 0;
      ASSERT_THROW_CL(clGetPlatformInfo(_id, info, sizeof(name), name, &ret_param_size));
      return std::string(name, ret_param_size-1);
    }

    static std::vector<platform> get() {
      cl_platform_id ids[128];
      cl_uint n = 0;
      ASSERT_THROW_CL(clGetPlatformIDs(128, ids, &n));
      std::vector<platform> vp(n);
      for (cl_uint i=0; i<n; ++i)
	vp[i] = platform(ids[i]);
      return vp;
    }

    std::vector<device> get_devices(cl_device_type device_type=CL_DEVICE_TYPE_DEFAULT) const {
      cl_device_id ids[128];
      cl_uint n=0;
      ASSERT_THROW_CL(clGetDeviceIDs(_id, device_type, 128, ids, &n));
      std::vector<device> vd(n);
      for (cl_uint i=0; i<n; ++i)
	vd[i] = device(ids[i]);
      return vd;
    }

  protected:
  private:
    cl_platform_id _id;
  } ;  

  class context {
  public:
    context()
      : _ctx(0) {}

    context(const platform& p,
	    const device&   d)
      : _ctx(0) {
      const cl_context_properties props[3] = { CL_CONTEXT_PLATFORM, reinterpret_cast<cl_context_properties>(p.id()), 0 };
      const cl_device_id ids[1] = { d.id() };
      cl_int err(CL_SUCCESS);
      _ctx = clCreateContext(props, 1, ids, NULL, NULL, &err);
      ASSERT_THROW_CL(err);
    }

    operator cl_context() const { return _ctx; }

    ~context() {
      clReleaseContext(_ctx);
    }
    cl_context ctx() const { return _ctx; }
  protected:
  private:
    cl_context _ctx;
  } ;
  
  class queue {
  public:
    queue()
      : _queue(0) {}
    queue(const context& ctx,
	  const device&  dev)
      : _queue(0) {
      cl_int err(CL_SUCCESS);
      _queue = clCreateCommandQueue(ctx, dev.id(), 0, &err);
      ASSERT_THROW_CL(err);
    }

    operator cl_command_queue() const { return _queue; }

    ~queue() {
      ASSERT_THROW_CL(clReleaseCommandQueue(_queue));
    }
  protected:
  private:
    cl_command_queue _queue;
  } ;
  
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
	  _buf = clCreateBuffer(get_context(), CL_MEM_READ_WRITE, n*sizeof(complex_type), NULL, &err);
	  ASSERT_THROW_CL(err);
	  _norm = n;
	}

	cl_context get_context() const {
	  cl_context ctx;
	  size_t n(0);
	  ASSERT_THROW_CL(clGetMemObjectInfo(_buf, CL_MEM_CONTEXT, sizeof(ctx), &ctx, &n));
	  ASSERT_THROW(n == sizeof(ctx));
	  return ctx;
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
	
	template<typename V,
		 template<typename U> class WINDOW_FCN>
	float fill(const std::vector<std::complex<V> >& v,
		   const WINDOW_FCN<V>& window_fcn) {
	  return fill(v.begin(), v.end(), window_fcn);
	}

	template<typename V,
		 template<typename U> class WINDOW_FCN>
	float fill(typename std::vector<std::complex<V> >::const_iterator i0,
		   typename std::vector<std::complex<V> >::const_iterator i1,
		   const WINDOW_FCN<V>& window_fcn) {
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
	
	ASSERT_THROW_CL(clfftSetPlanPrecision(_planHandle, CLFFT_SINGLE));
	ASSERT_THROW_CL(clfftSetLayout(_planHandle, CLFFT_COMPLEX_INTERLEAVED, CLFFT_COMPLEX_INTERLEAVED));
	ASSERT_THROW_CL(clfftSetResultLocation(_planHandle, CLFFT_INPLACE));
	
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
	cl_command_queue qs[1]     = { q };
	cl_mem           ms_in[1]  = { _in };
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

struct CL_Global {
  static void setup(std::string p="NVIDIA CUDA",
		    std::string d="GeForce 9800 GT") {    
    const std::vector<CL::platform> ps = CL::platform::get();
    size_t i=0;
    bool found(false);
    for (found=false; i<ps.size() && !found; ++i)
      found = (ps[i].info() == p);

    if (!found) {
      std::cout << "Error: platform not found\n";
      // throw error
    }
    --i;
    std::cout << "platform: " << i << " " << ps[i].info() << std::endl;

    CL_Global::platform() = ps[i];

    const std::vector<CL::device> ds = ps[i].get_devices(CL_DEVICE_TYPE_ALL);
    size_t j=0;
    
    for (found=false; j<ds.size() && !found; ++j)
      found = (ds[j].name() == d);

    if (!found) {
      std::cout << "Error: device not found\n";
      // throw error
    }
    --j;
    std::cout << "devices: " << j << " " << ds[j].name() << std::endl;
    CL_Global::device() = ds[j];
    
  }
  
  static CL::platform& platform() {
    static CL::platform p_;
    return p_;
  }
  
  static CL::device& device() {
    static CL::device device_;
    return device_;
  }
  
} ;

int main() {  
  const std::vector<CL::platform> ps = CL::platform::get();
  for (size_t i=0; i<ps.size(); ++i)
    std::cout << "platform[" << i << "] '" << ps[i].info() << "'" << std::endl;

  const std::vector<CL::device> ds = ps[0].get_devices(CL_DEVICE_TYPE_ALL);
  for (size_t i=0; i<ds.size(); ++i)
    std::cout << "dev[" << i <<"] '" << ds[i].name() << "'" << std::endl;

  CL_Global::setup();

  CL::context ctx(CL_Global::platform(), CL_Global::device());
  CL::queue queue(ctx, CL_Global::device());

  CL::FFT::setup setup;

  // CL::FFT::internal::Array in(ctx, 1024*1024*10);
  // std::cout << "A\n";
  // in.host_to_device(queue);
  // std::cout << "A\n";
  // in.device_to_host(queue);
  // std::cout << "A\n";

  const size_t n=8000;
  CL::FFT::clFFT fft(ctx, queue, n);

  FFT::FFTWTransform<float> fftw(n, FFTW_FORWARD, FFTW_ESTIMATE);

  for (int i=0; i<n; ++i) {
    fftw.in(i) = fft.in(i) = std::complex<float>(cos(10.*float(i)/float(n)*2*M_PI),
						 sin(-10.*float(i)/float(n)*2*M_PI));
  }
  fft.transform(queue);
  fftw.transform();

  for (int i=0; i<1024; ++i) {
    std::cout << std::abs(fft.out(i)-fftw.out(i)) << " " << fft.out(i) << " " << fftw.out(i) << std::endl;
  }

  return EXIT_SUCCESS;  
}
