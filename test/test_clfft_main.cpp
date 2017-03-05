#include <iostream>
#include <stdlib.h>

#include "FFT.hpp"

#include "cl/fft/overlap_save.hpp"

#include "filter/fir.hpp"
#include "filter/fir/overlap_save.hpp"


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
    cl::fft::setup cl_fft_setup;

    const int l = 500*1000;   //10*8192;
    const int p = 125*1000+1; //10*1024+1;
    cl::fft::overlap_save_setup os1_setup;
    filter::fir::overlap_save::sptr os1 = os1_setup.make_overlap_save(l, p);

    filter::fir::overlap_save_setup os2_setup;
    filter::fir::overlap_save::sptr os2 = os2_setup.make_overlap_save(l, p);
    filter::fir::lowpass<float> fir(os1->p());

    fir.design(0.1, 0.02);

    const int decim = 10;
    {
      auto const xx1 = os1->add_filter(fir.coeff(), 0.5, decim);
      std::cout << "xx1= (" << xx1.first << ", " << xx1.second << ")"<< std::endl;
      auto const xx2 = os2->add_filter(fir.coeff(), 0.5, decim);
      std::cout << "xx2= (" << xx2.first << ", " << xx2.second << ")"<< std::endl;
    }
    {
      auto const xx1 = os1->add_filter(fir.coeff(), 0.22, decim);
      std::cout << "xx1= (" << xx1.first << ", " << xx1.second << ")"<< std::endl;
      auto const xx2 = os2->add_filter(fir.coeff(), 0.22, decim);
      std::cout << "xx2= (" << xx2.first << ", " << xx2.second << ")"<< std::endl;
    }

    // generate data
    typedef cl::fft::overlap_save::complex_type complex_type;

    const int N = 5*l;
    processor::base_iq::vector_type buf1(N), buf2(N);
    const float f = 0.22029*l;
    for (int i=0; i<N; ++i) {
      const float t = float(i)/float(l);
      buf1[i] = buf2[i] = complex_type(std::cos(2*M_PI*f*t), std::sin(2*M_PI*f*t));
    }
    for (int j=0; j<5; ++j) {
      os1->proc(buf1.begin()+j*l, buf1.begin()+j*l+l);
      os2->proc(buf2.begin()+j*l, buf2.begin()+j*l+l);
      {
	const std::vector<complex_type> r1(os1->begin(1), os1->end(1));
	const std::vector<complex_type> r2(os2->begin(1), os2->end(1));
	for (int i=0; i<3; ++i)
	  std::cout << i << " " << r1[0*(p-1)/decim+i] << " " << r2[i] << " " << std::abs(r1[i]-r2[i]) << std::endl;
	for (int i=r2.size()-3,n=r2.size(); i<n; ++i)
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
