#include <iostream>

#include <stdexcept>
#include <sstream>
#include <boost/current_function.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/cstdint.hpp>

#include <portaudio.h>

#define THROW_SITE_INFO(what)                                           \
  std::string(std::string(what) + "\n" +                                \
              "  in " + std::string(BOOST_CURRENT_FUNCTION) + "\n" +    \
              "  at " + std::string(__FILE__) + ":" +			\
	      BOOST_STRINGIZE(__LINE__) + "\n")

#define ASSERT_THROW_PA(_x) {						\
    const PaError __pa_error_code__((_x));				\
    if (__pa_error_code__ < paNoError)	{				\
      throw std::runtime_error(THROW_SITE_INFO("assertion failed: "	\
					       + std::string(#_x) + " '"	\
					       + std::string(Pa_GetErrorText(__pa_error_code__))+"'")); \
    } else void(0); }

namespace portaudio {
  class init : public boost::noncopyable {
  public:
    init() { 
      if (not is_initialized()) {
	ASSERT_THROW_PA(Pa_Initialize());
	is_initialized() = true;
      }
    }
    ~init() { 
      if (is_initialized()) {
	ASSERT_THROW_PA(Pa_Terminate());
	is_initialized() = false;
      }
    }
  private:
    static bool& is_initialized() { static bool b(false); return b; }
  } ;

  class device_info : public boost::noncopyable {
  public:
    virtual ~device_info() {}
    typedef boost::shared_ptr<device_info> sptr;

    static sptr make(const PaDeviceInfo*);

    virtual std::string to_string() const = 0;

    virtual std::string name() const = 0;
    virtual PaHostApiIndex api_host_index() const = 0;
    virtual int max_input_channels() const = 0;
    virtual int max_output_channels() const = 0;
    virtual PaTime default_low_input_latency() const = 0;
    virtual PaTime default_low_output_latency() const = 0;
    virtual PaTime default_high_input_latency() const = 0;
    virtual PaTime default_high_output_latency() const = 0;
    virtual double default_sample_rate() const = 0;
  } ;

  class device_info_impl : public device_info {
  public:
    device_info_impl(const PaDeviceInfo* device_info) 
      : _device_info(device_info) {}

    std::string to_string() const {
      std::ostringstream oss;
      oss << "name= '" << name()  << "'"
	  << " api_host_index= " << api_host_index()
	  << " max_input_channels= " << max_input_channels()
	  << " max_output_channels= " << max_output_channels()
	  << " default_low_input_latency= " << default_low_input_latency()
	  << " default_low_output_latency= " << default_low_output_latency()
	  << " default_high_input_latency= " << default_high_input_latency()
	  << " default_high_output_latency= " << default_high_output_latency()
	  << " default_sample_rate= " << default_sample_rate();
      return oss.str();
    }

    virtual std::string name() const { return _device_info->name ; }
    virtual PaHostApiIndex api_host_index() const { return _device_info->hostApi ; }
    virtual int max_input_channels() const { return _device_info->maxInputChannels ; }
    virtual int max_output_channels() const { return _device_info->maxOutputChannels ; }
    virtual PaTime default_low_input_latency() const { return _device_info->defaultLowInputLatency ; }
    virtual PaTime default_low_output_latency() const { return _device_info->defaultLowOutputLatency ; }
    virtual PaTime default_high_input_latency() const { return _device_info->defaultHighInputLatency ; }
    virtual PaTime default_high_output_latency() const { return _device_info->defaultHighOutputLatency ; }
    virtual double default_sample_rate() const { return _device_info->defaultSampleRate ; }

  private:
    const PaDeviceInfo* _device_info; 
  } ;

  device_info::sptr device_info::make(const PaDeviceInfo* p) {
    return device_info::sptr(new device_info_impl(p));
  }

} // namespace portaudio


int main()
{
  try {
    portaudio::init pa;
    PaDeviceIndex numDevices;
    ASSERT_THROW_PA(numDevices=Pa_GetDeviceCount());
    
    for(int i=0; i<numDevices; i++ ) {
      portaudio::device_info::sptr di(portaudio::device_info::make(Pa_GetDeviceInfo(i)));
      std::cout << "i= " << di->to_string() << std::endl;
    }
    std::cout << Pa_GetVersion() << std::endl;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 0;
  }
  return 1;
}
