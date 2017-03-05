// -*- C++ -*-

#ifndef CL_ERROR_cm20170304_
#define CL_ERROR_cm20170304_

#include <boost/config.hpp> // BOOST_STRINGIZE
#include <boost/current_function.hpp>

#include <clFFT.h>

namespace cl {
  struct error {
    static const char* to_string(cl_int err) {
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
} // namespace cl

#ifndef THROW_SITE_INFO
#define THROW_SITE_INFO(what)                                           \
  std::string(std::string(what) + "\n" +                                \
	      "  in " + std::string(BOOST_CURRENT_FUNCTION) + "\n" +    \
	      "  at " + std::string(__FILE__) + ":" +                   \
	      BOOST_STRINGIZE(__LINE__) + "\n")
#endif // THROW_SITE_INFO

#ifndef ASSERT_THROW
#define ASSERT_THROW(_x)                                                \
  if (not (_x))                                                         \
    throw std::runtime_error(THROW_SITE_INFO("assertion failed: " + std::string(#_x))); \
  else void(0)
#endif // ASSERT_THROW

#define ASSERT_THROW_CL(_x) {						\
    const cl_int __cl_error_code__((_x));				\
    if (__cl_error_code__ != CL_SUCCESS)  {				\
      throw std::runtime_error(THROW_SITE_INFO("assertion failed: "     \
					       + std::string(#_x) + " '" \
					       + std::string(cl::error::to_string(__cl_error_code__))+"'")); \
    } else void(0); }

#endif // CL_ERROR_cm20170304_
