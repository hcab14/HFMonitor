// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2011 Christoph Mayer
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#ifndef _portaudio_hpp_cm110307_
#define _portaudio_hpp_cm110307_
#include <iostream>
#include <vector>
#include <stdexcept>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

#include <portaudio.h>

namespace portaudio {
  class input_overflow : public std::runtime_error {
  public:
    input_overflow(std::string msg) 
      : std::runtime_error(msg) {}
  } ;

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

  class session : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<session> sptr;

    typedef std::vector<device_info::sptr> device_list;

    virtual ~session() {}
    static sptr make();
    static sptr get_global_session();

    virtual int version_number() const = 0;
    virtual std::string version_text() const = 0; 
    virtual device_list get_device_list(std::string name="") const = 0;
    virtual bool is_format_supported(stream_parameters::sptr input_parameters,
                                     stream_parameters::sptr output_parameters,
                                     double sample_rate) const = 0;
    virtual void sleep(double seconds) const = 0;
  } ;

  class callback_info : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<callback_info> sptr;
    virtual ~callback_info() {}
    
    virtual PaTime input_buffer_adc_time() const = 0;
    virtual PaTime current_time() const = 0;
    virtual PaTime ouput_buffer_dac_time() const = 0;
    virtual bool input_underflow() const = 0;
    virtual bool input_overflow() const = 0;
    virtual bool output_underflow() const = 0;
    virtual bool output_overflow() const = 0;
    virtual bool priming_output() const = 0;

    virtual std::string to_string() const = 0;
  } ;

  class stream : public boost::noncopyable {
  public:
    virtual ~stream() {}

    virtual PaStream* get() = 0;
    virtual stream_info::sptr get_info() const = 0;

    virtual void abort() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;

    virtual bool is_stopped() const = 0;
    virtual bool is_active() const = 0;
  } ;


  class stream_callback : public stream {
  public:
    typedef boost::shared_ptr<stream_callback> sptr;
    virtual ~stream_callback() {}

    class process_base {
    public:
      typedef boost::shared_ptr<process_base> sptr;
      virtual ~process_base() {}
      virtual bool is_running() const { return true; }
      virtual int process(const void *input_buffer, 
                          void *output_buffer,
                          unsigned long frames_per_buffer,
                          callback_info::sptr info) {
        std::cout << "process_base callback called " << frames_per_buffer << " " << info->to_string() << std::endl;
        return paContinue;
      }
    } ;

    static sptr make(stream_parameters::sptr input_parameters,
                     stream_parameters::sptr output_parameters,
                     double sample_rate,
                     unsigned long frames_per_buffer,
                     PaStreamFlags stream_flags,
                     process_base::sptr p);

    virtual PaStream* get() = 0;
    virtual stream_info::sptr get_info() const = 0;

    virtual signed long get_write_available() const = 0;

    virtual PaTime get_time() const = 0;
    virtual double get_cpu_load() const = 0;

    virtual process_base::sptr get_processor() = 0;
  } ;

  class stream_blocking : public stream {
  public:
    typedef boost::shared_ptr<stream_blocking> sptr;
    virtual ~stream_blocking() {}

    static sptr make(stream_parameters::sptr input_parameters,
                     stream_parameters::sptr output_parameters,
                     double sample_rate,
                     unsigned long frames_per_buffer,
                     PaStreamFlags stream_flags);

    virtual void read_data(void* buffer, unsigned long frames) = 0;
    virtual void write_data(const void* buffer, unsigned long frames) = 0;
  } ;

} // namespace portaudio
#endif // _portaudio_hpp_cm110307_
