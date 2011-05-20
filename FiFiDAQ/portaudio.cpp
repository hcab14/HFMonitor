// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>

#include <stdexcept>
#include <sstream>
#include <boost/format.hpp>
#include <boost/current_function.hpp>

#include "portaudio.hpp"

#define THROW_SITE_INFO(what)                                           \
  std::string(std::string(what) + "\n" +                                \
              "  in " + std::string(BOOST_CURRENT_FUNCTION) + "\n" +    \
              "  at " + std::string(__FILE__) + ":" +                   \
              BOOST_STRINGIZE(__LINE__) + "\n")

#define ASSERT_THROW_PA(_x) {                                           \
    const PaError __pa_error_code__((_x));                              \
    if (__pa_error_code__ < paNoError)  {                               \
      throw std::runtime_error(THROW_SITE_INFO("assertion failed: "     \
                                               + std::string(#_x) + " '" \
                                               + std::string(Pa_GetErrorText(__pa_error_code__))+"'")); \
    } else void(0); }

namespace portaudio {
  class init_impl : public init {
  public:
   init_impl() { 
      if (not is_initialized()) {
        ASSERT_THROW_PA(Pa_Initialize());
        is_initialized() = true;
      }
    }
    ~init_impl() { 
      if (is_initialized()) {
        ASSERT_THROW_PA(Pa_Terminate());
        is_initialized() = false;
      }
    }
    virtual int version_number() const { return Pa_GetVersion(); }
    virtual std::string version_text() const { return Pa_GetVersionText(); }
    virtual device_list get_device_list(std::string name) const {
      device_list result;
      const PaDeviceIndex numDevices(Pa_GetDeviceCount());
      for(int i=0; i<numDevices; i++) {
        portaudio::device_info::sptr dp(portaudio::device_info::make(Pa_GetDeviceInfo(i), i));
        if (name == "" || name == dp->name())
          result.push_back(dp);
      }
      return result;
    }
    virtual bool is_format_supported(stream_parameters::sptr input_parameters,
                                     stream_parameters::sptr output_parameters,
                                     double sample_rate) const {
      PaError err(paNoError);
      ASSERT_THROW_PA(err=Pa_IsFormatSupported((input_parameters  == 0) ? 0 : input_parameters->get(),
                                               (output_parameters == 0) ? 0 : output_parameters->get(),
                                               sample_rate));
      return (err == paNoError);
    }
    virtual void sleep(double seconds) const {
      Pa_Sleep(1000*seconds);
    }

  private:
    bool& is_initialized(bool ib=false) { static bool b(ib); return b; }    
  } ;

  init::sptr init::make() {
    return init::sptr(new init_impl());
  }

  class device_info_impl : public device_info {
  public:
    device_info_impl(const PaDeviceInfo* device_info,
                     PaDeviceIndex index) 
      : _device_info(device_info)
      , _index(index) {}

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
          << " default_sample_rate= " << default_sample_rate() 
          << " index= " << _index;
      return oss.str();
    }

    virtual std::string name() const { return _device_info->name; }
    virtual PaHostApiIndex api_host_index() const { return _device_info->hostApi; }
    virtual int max_input_channels() const { return _device_info->maxInputChannels; }
    virtual int max_output_channels() const { return _device_info->maxOutputChannels; }
    virtual PaTime default_low_input_latency() const { return _device_info->defaultLowInputLatency; }
    virtual PaTime default_low_output_latency() const { return _device_info->defaultLowOutputLatency; }
    virtual PaTime default_high_input_latency() const { return _device_info->defaultHighInputLatency; }
    virtual PaTime default_high_output_latency() const { return _device_info->defaultHighOutputLatency; }
    virtual double default_sample_rate() const { return _device_info->defaultSampleRate; }

    virtual PaDeviceIndex index() const { return _index; }
  private:
    const PaDeviceInfo* _device_info;
    const PaDeviceIndex _index;
  } ;

  device_info::sptr device_info::make(const PaDeviceInfo* p, PaDeviceIndex index) {
    return device_info::sptr(new device_info_impl(p, index));
  }

  class stream_info_impl : public stream_info {
  public:
    stream_info_impl(const PaStreamInfo* pa_stream_info)
      : _pa_stream_info(pa_stream_info) {}

    virtual PaTime input_latency() const { return _pa_stream_info->inputLatency; }
    virtual PaTime output_latency() const { return _pa_stream_info->outputLatency; }
    virtual double sample_rate() const { return _pa_stream_info->sampleRate; }

  private:
    const PaStreamInfo* _pa_stream_info;
  } ;
  
  class stream_parameters_impl : public stream_parameters {
  public:
    stream_parameters_impl(PaDeviceIndex device_index,
                           int channel_count,
                           PaSampleFormat sample_format,
                           PaTime pa_suggested_latency) {
      _pa_stream_parameters.device = device_index;
      _pa_stream_parameters.channelCount = channel_count;
      _pa_stream_parameters.sampleFormat = sample_format;
      _pa_stream_parameters.suggestedLatency = pa_suggested_latency;
      _pa_stream_parameters.hostApiSpecificStreamInfo = 0;
    }

    virtual PaDeviceIndex device_index() const { return _pa_stream_parameters.device; }
    virtual int channel_count() const { return _pa_stream_parameters.channelCount; }
    virtual PaSampleFormat sampleFormat() const { return _pa_stream_parameters.sampleFormat; }
    virtual PaTime suggested_latency() const { return _pa_stream_parameters.suggestedLatency; }

    virtual const PaStreamParameters* get() const { return &_pa_stream_parameters; }
  private:
    PaStreamParameters _pa_stream_parameters;
  } ;

  stream_parameters::sptr
  stream_parameters::make(PaDeviceIndex device_index,
                          int channel_count,
                          PaSampleFormat sample_format,
                          PaTime suggested_latency) {
    return 
      stream_parameters::sptr(new stream_parameters_impl(device_index,
                                                         channel_count,
                                                         sample_format,
                                                         suggested_latency));
  }

  class callback_info_impl : public callback_info {
  public:
    callback_info_impl(const PaStreamCallbackTimeInfo* p,
                       PaStreamCallbackFlags flags)
      : _current_time(p->currentTime)
      , _input_buffer_adc_time(p->inputBufferAdcTime)
      , _ouput_buffer_dac_time(p->outputBufferDacTime)
      , _flags(flags) {}
    
    virtual PaTime input_buffer_adc_time() const { return _input_buffer_adc_time; }
    virtual PaTime current_time() const { return _current_time; }
    virtual PaTime ouput_buffer_dac_time() const { return _ouput_buffer_dac_time; }
    virtual bool input_underflow() const { return _flags & paInputUnderflow; }
    virtual bool input_overflow() const { return _flags & paInputOverflow; }
    virtual bool output_underflow() const { return _flags & paOutputUnderflow; }
    virtual bool output_overflow() const { return _flags & paOutputOverflow; }
    virtual bool priming_output() const { return _flags & paPrimingOutput; }

    virtual std::string to_string() const {
      std::ostringstream oss;
      oss << (boost::format("%.3f") % current_time()) << " " 
          << (boost::format("%.3f") % input_buffer_adc_time()) << " "
          << (boost::format("%.3f") % ouput_buffer_dac_time()) << " "
          << (input_underflow() ? " input_underflow " : "")
          << (input_overflow() ? " input_overflow " : "")
          << (output_underflow() ? " output_underflow " : "")
          << (output_overflow() ? " output_overflow " : "")
          << (priming_output() ? " priming_output " : "");        
      return oss.str();
    }
  private:
    const PaTime _current_time;
    const PaTime _input_buffer_adc_time;
    const PaTime _ouput_buffer_dac_time;
    const PaStreamCallbackFlags _flags;
  } ;

  class stream_callback_impl : public stream_callback {
  public:
    stream_callback_impl(stream_parameters::sptr input_parameters,
                         stream_parameters::sptr output_parameters,
                         double sample_rate,
                         unsigned long frames_per_buffer,
                         PaStreamFlags stream_flags,
                         process_base::sptr p) 
      : processor_(p) {
      ASSERT_THROW_PA(Pa_OpenStream(&pa_stream_,
                                    (input_parameters  == 0) ? 0 : input_parameters->get(),
                                    (output_parameters == 0) ? 0 : output_parameters->get(),
                                    sample_rate,
                                    frames_per_buffer, // paFramesPerBufferUnspecified
                                    stream_flags,
                                    stream_callback_impl::callback,
                                    this));
    }

    ~stream_callback_impl() { ASSERT_THROW_PA(Pa_CloseStream(pa_stream_)); }

    virtual PaStream* get() { return pa_stream_; }
    virtual stream_info::sptr get_info() const {
      return stream_info::sptr(new stream_info_impl(Pa_GetStreamInfo(pa_stream_)));
    }

    virtual void abort() { ASSERT_THROW_PA(Pa_AbortStream(pa_stream_)); }
    virtual void start() { ASSERT_THROW_PA(Pa_StartStream(pa_stream_)); }
    virtual void stop()  { ASSERT_THROW_PA(Pa_StopStream(pa_stream_)); }

    signed long get_write_available() const {
      signed long n(0);
      ASSERT_THROW_PA(n=Pa_GetStreamWriteAvailable(pa_stream_));
      return n;
    }
    virtual bool is_stopped() const {
      PaError err;
      ASSERT_THROW_PA(err=Pa_IsStreamStopped(pa_stream_));
      return (err == 1) ? true : false;
    }
    virtual bool is_active() const {
      PaError err;
      ASSERT_THROW_PA(err=Pa_IsStreamActive(pa_stream_));
      return (err == 1) ? true : false;
    }
    virtual PaTime get_time() const {
      const PaTime t(Pa_GetStreamTime(pa_stream_));
      if (t == 0) throw std::runtime_error(THROW_SITE_INFO("Pa_GetStreamTime returned zero"));
      return t;
    }
    virtual double get_cpu_load() const { 
      return Pa_GetStreamCpuLoad(pa_stream_); 
    }
    virtual process_base::sptr get_processor() {
      return processor_;
    }
  private:
    static int callback(const void *input_buffer, void *output_buffer,
                        unsigned long frames_per_buffer,
                        const PaStreamCallbackTimeInfo* time_info,
                        PaStreamCallbackFlags status_flags,
                        void *userData) {
      stream_callback_impl* sp(reinterpret_cast<stream_callback_impl*>(userData));
      const callback_info::sptr callback_info(new callback_info_impl(time_info, status_flags));
      return sp->get_processor()->process(input_buffer, output_buffer, frames_per_buffer, callback_info);
    }

    PaStream* pa_stream_;
    process_base::sptr processor_;
  } ;

  class stream_blocking_impl : public stream_blocking {
  public:
    stream_blocking_impl(stream_parameters::sptr input_parameters,
                         stream_parameters::sptr output_parameters,
                         double sample_rate,
                         unsigned long frames_per_buffer,
                         PaStreamFlags stream_flags) {
      ASSERT_THROW_PA(Pa_OpenStream(&pa_stream_,
                                    (input_parameters  == 0) ? 0 : input_parameters->get(),
                                    (output_parameters == 0) ? 0 : output_parameters->get(),
                                    sample_rate,
                                    frames_per_buffer, // paFramesPerBufferUnspecified
                                    stream_flags, 
                                    0,0));
    }

    ~stream_blocking_impl() { ASSERT_THROW_PA(Pa_CloseStream(pa_stream_)); }

    virtual PaStream* get() { return pa_stream_; }
    virtual stream_info::sptr get_info() const {
      return stream_info::sptr(new stream_info_impl(Pa_GetStreamInfo(pa_stream_)));
    }

    virtual void abort() { ASSERT_THROW_PA(Pa_AbortStream(pa_stream_)); }
    virtual void start() { ASSERT_THROW_PA(Pa_StartStream(pa_stream_)); }
    virtual void stop() { ASSERT_THROW_PA(Pa_StopStream(pa_stream_)); }

    virtual bool is_stopped() const {
      PaError err;
      ASSERT_THROW_PA(err=Pa_IsStreamStopped(pa_stream_));
      return (err == 1) ? true : false;
    }
    virtual bool is_active() const {
      PaError err;
      ASSERT_THROW_PA(err=Pa_IsStreamActive(pa_stream_));
      return (err == 1) ? true : false;
    }
    virtual void read_data(void* buffer, unsigned long frames) {
      if (Pa_ReadStream(pa_stream_, buffer, frames) == paInputOverflowed) 
        throw input_overflow(THROW_SITE_INFO("input overflow in Pa_ReadStream"));
    }
    virtual void write_data(const void* buffer, unsigned long frames) {
      ASSERT_THROW_PA(Pa_WriteStream(pa_stream_, buffer, frames));
    }

  private:
    PaStream* pa_stream_;
  } ;

  stream_callback::sptr 
  stream_callback::make(stream_parameters::sptr input_parameters,
                        stream_parameters::sptr output_parameters,
                        double sample_rate,
                        unsigned long frames_per_buffer,
                        PaStreamFlags stream_flags,
                        process_base::sptr p) {
    return 
      stream_callback::sptr(new stream_callback_impl(input_parameters,
                                                     output_parameters,
                                                     sample_rate,
                                                     frames_per_buffer,
                                                     stream_flags,
                                                     p));
  }

  stream_blocking::sptr 
  stream_blocking::make(stream_parameters::sptr input_parameters,
                        stream_parameters::sptr output_parameters,
                        double sample_rate,
                        unsigned long frames_per_buffer,
                        PaStreamFlags stream_flags) {
    return 
      stream_blocking::sptr(new stream_blocking_impl(input_parameters,
                                                     output_parameters,
                                                     sample_rate,
                                                     frames_per_buffer,
                                                     stream_flags));
  }

} // namespace portaudio
