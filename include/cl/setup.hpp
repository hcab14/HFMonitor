// -*- C++ -*-

#ifndef _CL_SETUP_HPP_cm170308_
#define _CL_SETUP_HPP_cm170308_

#include <vector>

#define CL_USE_DEPRECATED_OPENCL_2_0_APIS
#define __CL_ENABLE_EXCEPTIONS
#include "cl/cl.hpp"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "logging.hpp"

namespace cl {
  class setup {
  public:
    setup()
      : _queue(make_queue()) {}

    cl::CommandQueue& queue() { return _queue; }

  protected:
    static cl::CommandQueue make_queue() {
      std::vector<cl::Platform> platforms;
      cl::Platform::get(&platforms);
      ASSERT_THROW(!platforms.empty());

      auto const& default_platform = platforms[0];
      std::cout << "Using platform: "<< default_platform.getInfo<CL_PLATFORM_NAME>() << " #platforms=" << platforms.size() << "\n";

      cl_context_properties properties[] =
	{ CL_CONTEXT_PLATFORM, (cl_context_properties)(platforms[0])(), 0};
      cl::Context ctx(CL_DEVICE_TYPE_GPU, properties);

      std::vector<cl::Device> devices = ctx.getInfo<CL_CONTEXT_DEVICES>();
      ASSERT_THROW(!devices.empty());

      auto const& default_device = devices[0];
      return cl::CommandQueue(ctx, default_device);
      }

  private:
    cl::CommandQueue _queue;
  } ;
} // namespace cl

#endif // _CL_SETUP_HPP_cm170308_
