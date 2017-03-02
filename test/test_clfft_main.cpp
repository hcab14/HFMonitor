#include <iostream>
#include <fstream>
#include <sstream>
#include <complex>
#include <vector>
#include <unistd.h>
#include <stdlib.h>

#ifdef USE_OPENCL
#  include "cl/cl.hpp"
#  include "cl/FFT.hpp"
#endif
#include "FFT.hpp"

int main() {  
#ifdef USE_OPENCL
  const std::vector<CL::platform> ps = CL::platform::get();
  for (size_t i=0; i<ps.size(); ++i)
    std::cout << "platform[" << i << "] '" << ps[i].info() << "'" << std::endl;

  const std::vector<CL::device> ds = ps[0].get_devices(CL_DEVICE_TYPE_ALL);
  for (size_t i=0; i<ds.size(); ++i)
    std::cout << "dev[" << i <<"] '" << ds[i].name() << "'" << std::endl;

  CL::Global::setup("AMD Accelerated Parallel Processing",
		   "Hainan");

  CL::context ctx(CL::Global::platform(), CL::Global::device());
  CL::queue queue(ctx, CL::Global::device());

#if 1
  std::ifstream t("test/test.cl");
  std::stringstream buffer;
  buffer << t.rdbuf();

  CL::program p(ctx, buffer.str());
  p.build("-Werror");

  CL::kernel k(p, "simple_add");
  std::cout << "kernel name=" << k.name() << std::endl;

  CL::context ctx2 = k.context(); // for test

  CL::mem buf(ctx, 100*sizeof(float));
  std::vector<float> vv1(100), vv2(100);
  for (int i=0; i<100; ++i)
    vv1[i]=i;

  std::vector<CL::event> event_list;
  CL::event event = queue.enqueueWriteBuffer(buf, 100*sizeof(float), &vv1[0], 0, event_list);
  event_list.push_back(event);
  event = queue.enqueueReadBuffer (buf, 100*sizeof(float), &vv2[0], 0, event_list);
  event_list.push_back(event);
  CL::Global::waitForEvents(event_list);

  queue.finish();

  for (int i=0; i<10; ++i)
    std::cout << "vv1,2 =" << vv1[i] << " " << vv2[i] << std::endl;
#endif
  {
    CL::FFT::setup setup;
    
    const size_t n=16;
    CL::FFT::clFFT fft(ctx, queue, n);

    FFT::FFTWTransform<float> fftw(n, FFTW_FORWARD, FFTW_ESTIMATE);
    
    for (size_t i=0; i<n; ++i)
      fftw.in(i) = fft.in(i) = std::complex<float>(i, 3*i);

    fft.transform(queue);
    fftw.transform();

    for (size_t i=0; i<n; ++i)
      std::cout << i << " " << std::abs(fft.out(i)-fftw.out(i)) << " " << fft.out(i) << " " << fftw.out(i) << std::endl;

  }
#endif // USE_OPENCL
  return EXIT_SUCCESS;  
}