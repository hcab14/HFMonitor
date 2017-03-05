#ifndef _CL_FFT_HPP_cm170305_
#define _CL_FFT_HPP_cm170305_

#ifdef USE_OPENCL
#  include <clFFT.h>
#  define __CL_ENABLE_EXCEPTIONS
#  include "cl/cl.hpp"
#  include "cl/array.hpp"
#  include "cl/error.hpp"
#endif

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
  } // namespace fft
} // namespace cl

#endif // _CL_FFT_HPP_cm170305_
