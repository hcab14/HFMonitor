#ifndef _CL_CL_HPP_cm160419_
#define _CL_CL_HPP_cm160419_

#include <vector>

#include <boost/current_function.hpp>
#include <boost/format.hpp>
#define CL_USE_DEPRECATED_OPENCL_1_1_APIS
#define CL_USE_DEPRECATED_OPENCL_2_0_APIS
#include <CL/cl.h>

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
    typedef cl_device_id id_type;
    device(cl_device_id id=0)
      : _id(id) {}

    operator cl_device_id() const { return _id; }

    std::string name() const {
      char device_name[128];
      size_t n(0);
      ASSERT_THROW_CL
	(clGetDeviceInfo(_id, CL_DEVICE_NAME, sizeof(device_name), device_name, &n));
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

    operator cl_platform_id() const { return _id; }

    std::string info(cl_platform_info info=CL_PLATFORM_NAME) const {
      char name[1024];
      size_t ret_param_size(0);
      ASSERT_THROW_CL
	(clGetPlatformInfo(_id, info, sizeof(name), name, &ret_param_size));
      return std::string(name, ret_param_size-1);
    }

    static std::vector<platform> get() {
      cl_platform_id ids[128];
      cl_uint n(0);
      ASSERT_THROW_CL
	(clGetPlatformIDs(128, ids, &n));
      std::vector<platform> vp(n);
      for (cl_uint i=0; i<n; ++i)
	vp[i] = platform(ids[i]);
      return vp;
    }

    std::vector<device> get_devices(cl_device_type device_type=CL_DEVICE_TYPE_DEFAULT) const {
      cl_device_id ids[128];
      cl_uint n(0);
      ASSERT_THROW_CL
	(clGetDeviceIDs(_id, device_type, 128, ids, &n));
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
    context(cl_context c)
      : _context(c) {
      ASSERT_THROW_CL(clRetainContext(_context));      
    }
    context(const platform& p,
	    const device&   d)
      : _context(make(p, d)) {}
    context(const context& p)
      : _context(p._context) {
      ASSERT_THROW_CL(clRetainContext(_context));
    }
    ~context() {
      clReleaseContext(_context);
    }
    context& operator=(const context& p) {
      _context = p._context;
      ASSERT_THROW_CL(clRetainContext(_context));
      return *this;
    }

    operator cl_context() const { return _context; }

  protected:
    static cl_context make(const platform& p,
			   const device&   d) {
      const cl_context_properties props[3] = { 
	CL_CONTEXT_PLATFORM,
	reinterpret_cast<cl_context_properties>(static_cast<cl_platform_id>(p)),
	0
      };
      const cl_device_id ids[1] = { d };
      cl_int err(CL_SUCCESS);
      cl_context c = clCreateContext(props, 1, ids, NULL, NULL, &err);
      ASSERT_THROW_CL(err);
      return c;
    }
  private:
    cl_context _context;
  } ;

  namespace {
    // unpack a vector of class instances into a vector of the wrapped class member
    template<typename V>
    std::vector<typename V::value_type::id_type> make_vector(const V& v) {
    const size_t n(v.size());
    typedef typename V::value_type::id_type id_type;
    std::vector<id_type> v_id(n);
    for (size_t i(0); i<n; ++i)
      v_id[i] = id_type(v[i]);
    return v_id;
    }
  } // anonymous namespace

  class program {
  public:
    typedef cl_program id_type;
    program(cl_program p)
      : _program(p) {
      ASSERT_THROW_CL(clRetainProgram(_program));
    }
    program(context& ctx, std::string source)
      : _program(make(ctx, source)) {}
    program(context& ctx, std::vector<std::string> sources)
      : _program(make(ctx, sources)) {}
    program(const program& p)
      : _program(p._program) {
      ASSERT_THROW_CL(clRetainProgram(_program));
    }
    ~program() {
      clReleaseProgram(_program);
    }
    program& operator=(const program& p) {
      _program = p._program;
      ASSERT_THROW_CL(clRetainProgram(_program));
      return *this;
    }

    operator cl_program() const { return _program; }

    void build(std::string options) {
      std::vector<device> device_list;
      build(device_list, options);
    }
    void build(const std::vector<device>& device_list, std::string options) {
      const std::vector<cl_device_id> device_id_list(make_vector(device_list));
      ASSERT_THROW_CL
	(clBuildProgram(_program,
			device_id_list.size(),
			&device_id_list[0],
			options.c_str(),
			NULL, NULL));
    }

    static program link(context &ctx,
			const std::vector<device>& devices,
			const std::vector<program>& programs,
			std::string options) {
      const std::vector<cl_device_id> device_id_list(make_vector(devices));
      const std::vector<cl_program>  program_id_list(make_vector(programs));
      cl_int err(CL_SUCCESS);
      program p(clLinkProgram(ctx,
			      device_id_list.size(),  &device_id_list[0],
			      options.c_str(),
			      program_id_list.size(), &program_id_list[0],
			      NULL, NULL,
			      &err));
      ASSERT_THROW_CL(err);      
      return p;
    }

  protected:
    static cl_program make(context& ctx, std::string source) {
      std::vector<std::string> sources;
      sources.push_back(source);
      return make(ctx, sources);
    }    
    static cl_program make(context& ctx, std::vector<std::string> sources) {
      const cl_uint count(sources.size()); // number of lines
      std::vector<const char* > src(count);
      std::vector<size_t>       len(count);
      for (size_t i=0; i<count; ++i) {
	src[i] = sources[i].c_str();
	len[i] = sources[i].size();
      }
      cl_int err(CL_SUCCESS);
      cl_program p_id = clCreateProgramWithSource(ctx, count, &src[0], &len[0], &err);
      ASSERT_THROW_CL(err);
      return p_id;
    }
  private:
    cl_program _program;
  } ;

  class kernel {
  public:
    kernel(cl_program  program,
	   std::string name)
      : _kernel(make(program, name)) {}
    kernel(const kernel& p)
      : _kernel(p._kernel) {
      ASSERT_THROW_CL(clRetainKernel(_kernel));
    }
    ~kernel() {
      clReleaseKernel(_kernel);
    }
    kernel& operator=(const kernel& p) {
      _kernel = p._kernel;
      ASSERT_THROW_CL(clRetainKernel(_kernel));
      return *this;
    }

    std::string name() const {
      char name[1024];
      size_t n(0);
      ASSERT_THROW_CL
	(clGetKernelInfo (_kernel, CL_KERNEL_FUNCTION_NAME, 1024, name, &n));
      return std::string(name, n);
    }    
    CL::context context() const {
      cl_context c;
      size_t n(0);
      ASSERT_THROW_CL
	(clGetKernelInfo (_kernel, CL_KERNEL_CONTEXT, sizeof(cl_context), &c, &n));
      ASSERT_THROW(n == sizeof(cl_context));
      return CL::context(c);
    }

    template<typename T>
    void set_argument(cl_uint arg_index, const T& arg_value) {
      ASSERT_THROW_CL
	(clSetKernelArg(_kernel, arg_index, sizeof(T), &arg_value));
    }
    
  protected:
    static cl_kernel make(cl_program  program,
			  std::string name) {
      cl_int err(CL_SUCCESS);
      cl_kernel k(clCreateKernel(program, name.c_str(), &err));
      ASSERT_THROW_CL(err);
      return k;
    }
  private:
    cl_kernel _kernel;
  } ;

  class mem {
  public:
    mem(const context &ctx, size_t size, cl_mem_flags flags=CL_MEM_READ_WRITE)
      : _mem(make(ctx, size, flags)) {}
    ~mem() {
      ASSERT_THROW_CL(clReleaseMemObject(_mem));
    }

    operator cl_mem() const { return _mem; }

  protected:
    static cl_mem make(const context &ctx, size_t size, cl_mem_flags flags) {
      cl_int err(CL_SUCCESS);
      cl_mem m(clCreateBuffer(ctx, flags, size, NULL, &err));
      ASSERT_THROW_CL(err);
      return m;
    }
  private:
    mem(const mem& );
    mem& operator=(const mem& );
    cl_mem _mem;
  } ;

  class event {
  public:
    typedef cl_event id_type;
    event(cl_event e)
      : _event(e) {
      ASSERT_THROW_CL(clRetainEvent(_event));      
    }
    event(const event& e)
      : _event(e._event) {
      ASSERT_THROW_CL(clRetainEvent(_event));
    }
    ~event() {
      clReleaseEvent(_event);
    }
    event& operator=(const event& e) {
      _event = e._event;
      ASSERT_THROW_CL(clRetainEvent(_event));
      return *this;
    }

    operator cl_event() const { return _event; }
  protected:
  private:
    cl_event _event;
  } ;

  class queue {
  public:
    queue(const context& ctx,
	  const device&  dev)
      : _queue(0) {
      cl_int err(CL_SUCCESS);
      _queue = clCreateCommandQueue(ctx, dev, 0, &err);
      ASSERT_THROW_CL(err);
    }
    ~queue() {
      ASSERT_THROW_CL(clReleaseCommandQueue(_queue));
    }

    operator cl_command_queue() const { return _queue; }

    void enqueueBarrier() {
      ASSERT_THROW_CL(clEnqueueBarrier(_queue));
    }
    void enqueueWaitForEvents(const std::vector<CL::event>& wait_list) {
      const std::vector<cl_event> event_list(make_vector(wait_list));
      ASSERT_THROW_CL
	(clEnqueueWaitForEvents(_queue, event_list.size(), &event_list[0]));
    }

    // device -> host
    void enqueueReadBuffer(const mem& m, size_t len, void *ptr, size_t offset=0) {
      ASSERT_THROW_CL
	(clEnqueueReadBuffer(_queue, m, false, offset, len, ptr, 0, NULL, NULL));
    }
    CL::event enqueueReadBuffer(const mem& m, size_t len, void *ptr, size_t offset,
				const std::vector<CL::event>& wait_list) {
      const std::vector<cl_event> event_list(make_vector(wait_list));
      cl_event e;
      ASSERT_THROW_CL
	(clEnqueueReadBuffer(_queue, m, false, offset, len, ptr, event_list.size(), &event_list[0], &e));
      return CL::event(e);
    }
    // host -> device
    void enqueueWriteBuffer(const mem& m, size_t len, const void *ptr, size_t offset=0) {
      ASSERT_THROW_CL
	(clEnqueueWriteBuffer(_queue, m, false, offset, len, ptr, 0, NULL, NULL));
    }
    CL::event enqueueWriteBuffer(const mem& m, size_t len, const void *ptr, size_t offset,
				 const std::vector<CL::event>& wait_list) {
      const std::vector<cl_event> event_list(make_vector(wait_list));
      cl_event e;
      ASSERT_THROW_CL
	(clEnqueueWriteBuffer(_queue, m, false, offset, len, ptr, event_list.size(), &event_list[0], &e));
      return CL::event(e);
    }
    // device -> device
    void enqueueCopyBuffer(const mem& src, mem& dst, size_t len, size_t src_offset=0, size_t dst_offset=0) {
      ASSERT_THROW_CL
	(clEnqueueCopyBuffer(_queue, src, dst, src_offset, dst_offset, len, 0, NULL, NULL));
    }
    CL::event enqueueCopyBuffer(const mem& src, mem& dst, size_t len, size_t src_offset, size_t dst_offset,
				const std::vector<CL::event>& wait_list) {
      const std::vector<cl_event> event_list(make_vector(wait_list));
      cl_event e;
      ASSERT_THROW_CL
	(clEnqueueCopyBuffer(_queue, src, dst, src_offset, dst_offset, len, event_list.size(), &event_list[0], &e));
      return CL::event(e);
    }
    
  protected:
  private:
    queue(const context& );
    queue& operator=(const queue& );
    cl_command_queue _queue;
  } ;

  struct Global {
    static bool setup(std::string p="NVIDIA CUDA",
		      std::string d="GeForce 9800 GT") {    
      const std::vector<CL::platform> ps = CL::platform::get();
      size_t i(0);
      bool found(false);
      for (found=false; i<ps.size() && !found; ++i)
	found = (ps[i].info() == p);

      if (!found) {
	std::cout << "Error: platform not found\n";
	return found;
      }
      --i;
      std::cout << "platform: " << i << " " << ps[i].info() << std::endl;

      CL::Global::platform() = ps[i];

      const std::vector<CL::device> ds = ps[i].get_devices(CL_DEVICE_TYPE_ALL);
      size_t j(0);    
      for (found=false; j<ds.size() && !found; ++j)
	found = (ds[j].name() == d);

      if (!found) {
	std::cout << "Error: device not found\n";
	return found;
      }
      --j;
      std::cout << "devices: " << j << " " << ds[j].name() << std::endl;
      CL::Global::device() = ds[j];
      return true;
    }
  
    static CL::platform& platform() {
      static CL::platform p_;
      return p_;
    }
  
    static CL::device& device() {
      static CL::device device_;
      return device_;
    }  

    static void waitForEvents(const std::vector<CL::event>& wait_list) {
      const std::vector<cl_event> event_list(make_vector(wait_list));
      ASSERT_THROW_CL(clWaitForEvents(event_list.size(), &event_list[0]));
    }
  } ;

} // namespace CL

//#undef ASSERT_THROW_CL

#endif // _CL_CL_HPP_cm160419_
