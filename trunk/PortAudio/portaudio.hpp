#include <iostream>
#include <vector>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

#include <portaudio.h>

namespace portaudio {

  class device_info : public boost::noncopyable {
  public:
    virtual ~device_info() {}
    typedef boost::shared_ptr<device_info> sptr;

    static sptr make(const PaDeviceInfo*, PaDeviceIndex index);

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

    virtual PaDeviceIndex index() const = 0;
  } ;

  class init : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<init> sptr;

    typedef std::vector<device_info::sptr> device_list;

    virtual ~init() {}
    static sptr make();

    virtual int version_number() const = 0;
    virtual std::string version_text() const = 0; 
    virtual device_list get_device_list(std::string name="") const = 0;
  } ;

  class stream_info : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<stream_info> sptr;
    virtual ~stream_info() {}

    virtual PaTime input_latency() const = 0;
    virtual PaTime output_latency() const = 0;
    virtual double sample_rate() const = 0;
  } ;
  
  class stream_parameters : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<stream_parameters> sptr;
    virtual ~stream_parameters() {}

    static sptr make(PaDeviceIndex device_index= 0,
		     int channel_count = 0,
		     PaSampleFormat sample_format = 0,
		     PaTime suggested_latency = 0);

    virtual PaDeviceIndex device_index() const = 0;
    virtual int channel_count() const = 0;
    virtual PaSampleFormat sampleFormat() const = 0;
    virtual PaTime suggested_latency() const = 0;
    virtual const PaStreamParameters* get() const = 0;
  } ;


  class stream : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<stream> sptr;
    virtual ~stream() {}
    static sptr make(stream_parameters::sptr input_parameters,
		     stream_parameters::sptr output_parameters,
		     double sample_rate,
		     unsigned long frames_per_buffer,
		     PaStreamFlags stream_flags);

    virtual void abort() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual signed long get_write_available() const = 0;

    virtual bool is_stopped() const = 0;
    virtual bool is_active() const = 0;
    virtual PaTime get_time() const = 0;
    virtual double get_cpu_load() const = 0;
    virtual stream_info::sptr get_info() const = 0;
  } ;

} // namespace portaudio
