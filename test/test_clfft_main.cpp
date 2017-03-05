#include <iostream>
#include <fstream>
#include <sstream>
#include <complex>
#include <vector>
#include <unistd.h>
#include <stdlib.h>

#include "FFT.hpp"

#include "aligned_vector.hpp"
#include "filter/fir.hpp"
#include "filter/fir/overlap_save.hpp"

#include "cl/fft/overlap_save.hpp"

void print_device_info(const cl::Device& device) {
  std::cout<< "Using device: "                             << device.getInfo<CL_DEVICE_NAME>()
	   << "\n\t CL_DEVICE_VENDOR "                     << device.getInfo<CL_DEVICE_VENDOR>()
	   << "\n\t CL_DRIVER_VERSIOn "                    << device.getInfo<CL_DRIVER_VERSION>()
	   << "\n\t CL_DEVICE_VERSION "                    << device.getInfo<CL_DEVICE_VERSION>()
	   << "\n\t CL_DEVICE_PROFILE "                    << device.getInfo<CL_DEVICE_PROFILE>()
	   << "\n\t CL_DEVICE_EXTENSIONS "                 << device.getInfo<CL_DEVICE_EXTENSIONS>()
	   << "\n\t CL_DEVICE_MAX_COMPUTE_UNITS="          << device.getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>()
	   << "\n\t CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS="   << device.getInfo<CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS>()
	   << "\n\t CL_DEVICE_MAX_WORK_GROUP_SIZE="        << device.getInfo<CL_DEVICE_MAX_WORK_GROUP_SIZE>()
    //<< "\n\t CL_DEVICE_MAX_WORK_ITEM_SIZES="  << device.getInfo<CL_DEVICE_MAX_WORK_ITEM_SIZES>()
	   << "\n\t CL_DEVICE_MAX_CLOCK_FREQUENCY="        << device.getInfo<CL_DEVICE_MAX_CLOCK_FREQUENCY>()
	   << "\n\t CL_DEVICE_MAX_MEM_ALLOC_SIZE="         << device.getInfo<CL_DEVICE_MAX_MEM_ALLOC_SIZE>()
	   << std::endl;
}

int main() {
#ifdef USE_OPENCL
  try {
    // (1) get (ctx, queue)
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
    for (auto const& device : devices) {
      print_device_info(device);
    }

    auto const& default_device = devices[0];
    cl::CommandQueue queue(ctx, default_device);

    //
    // (2) input: (ctx, queue)
    //
    cl::fft::setup cl_fft_setup;

    const int l = 500*1000;   //10*8192;
    const int p = 125*1000+1; //10*1024+1;
    cl::fft::overlap_save os(l, p, ctx, queue);
    filter::fir::overlap_save<float> os_(l, p);
    filter::fir::lowpass<float> fir(os.p());

    fir.design(0.1, 0.02);

    const int decim = 10;
    {
      auto const xx1 = os.add_filter(fir.coeff(), 0.5, decim);
      std::cout << "xx1= (" << xx1.first << ", " << xx1.second << ")"<< std::endl;
      auto const xx2 = os_.add_filter(fir.coeff(), 0.5, decim);
      std::cout << "xx2= (" << xx2.first << ", " << xx2.second << ")"<< std::endl;
    }
    {
      auto const xx1 = os.add_filter(fir.coeff(), 0.22, decim);
      std::cout << "xx1= (" << xx1.first << ", " << xx1.second << ")"<< std::endl;
      auto const xx2 = os_.add_filter(fir.coeff(), 0.22, decim);
      std::cout << "xx2= (" << xx2.first << ", " << xx2.second << ")"<< std::endl;
    }

    // generate data
    typedef cl::fft::overlap_save::complex_type complex_type;
    typedef cl::fft::overlap_save::complex_vector_type complex_vector_type;
    const int N = 5*l;
    complex_vector_type buf(N);
    aligned_vector<std::complex<float> > buf2(N);
    const float f = 0.22*l;
    for (int i=0; i<N; ++i) {
      const float t = float(i)/float(l);
      buf[i] = buf2[i] = complex_type(std::cos(2*M_PI*f*t), std::sin(2*M_PI*f*t));
    }
    for (int j=0; j<5; ++j) {
      os.proc(buf.begin()+j*l,   buf.begin()+j*l+l);
      os_.proc(buf2.begin()+j*l, buf2.begin()+  j*l+l);
      {
	auto const& r1 = os.get_filter(1)->result();
	auto const& r2 = os_.get_filter(1)->result();
	for (int i=0; i<3; ++i)
	  std::cout << i << " " << r1[0*(p-1)/decim+i] << " " << r2[i] << " " << std::abs(r1[i]-r2[i]) << std::endl;
	for (int i=r2.size()-3; i<r2.size(); ++i)
	  std::cout << i << " " << r1[0*(p-1)/decim+i] << " " << r2[i] << " " << std::abs(r1[i]-r2[i]) << std::endl;
      }
    }
  } catch (const cl::Error& err) {
    std::cerr << err.what() << "(" << cl::error::to_string(err.err()) << ")" << std::endl;
    return EXIT_FAILURE;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
#endif
  return EXIT_SUCCESS;
}
