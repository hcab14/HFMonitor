// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <fstream>
#include <octave/oct.h>
#include <octave/Matrix.h>
#include <octave/ov.h>
#include <octave/load-save.h>
#include <octave/ls-mat5.h>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "util.hpp"
#include "libusb1.hpp"
#include "portaudio.hpp"
#include "fifi_control.hpp"
#include "FFT.hpp"
#include "Spectrum.hpp"

usb_device_handle::sptr find_usb_device(const boost::property_tree::ptree& config_fifi_daq) {
  const unsigned vendor_id (config_fifi_daq.get<unsigned>("<xmlattr>.vendorID",  0));
  const unsigned product_id(config_fifi_daq.get<unsigned>("<xmlattr>.productID", 0));
  const std::vector<usb_device_handle::sptr> dl(usb_device_handle::get_device_list(vendor_id, product_id));
  if (dl.size() != 1) {
    const std::vector<usb_device_handle::sptr> dl(usb_device_handle::get_device_list(0, 0));
    std::cerr << "Available USB devices:\n";
    for (size_t i=0; i<dl.size(); ++i) {
      std::cerr << (boost::format("(%2d) vendor_id=%5d product_id=%5d serial='%s'") 
                    % i
                    % dl[i]->get_vendor_id() 
                    % dl[i]->get_product_id()
                    % dl[i]->get_serial()) << std::endl;
    }
    throw std::runtime_error(str(boost::format("FiFi-SDR (vendor_id=%5d, product_id=%5d) not found.") 
                                 % vendor_id % product_id));
  } else
    return dl[0];
}

portaudio::device_info::sptr find_portaudio_device(const boost::property_tree::ptree& config_fifi_daq,
                                                   portaudio::init::sptr pa) {
  const std::string device_name(config_fifi_daq.get<std::string>("<xmlattr>.audioDeviceName", "UDA1361 Eingang"));
  const portaudio::init::device_list dl(pa->get_device_list(device_name));
  if (dl.size() != 1) {
    const portaudio::init::device_list dl(pa->get_device_list());
    std::cerr << "Audio Device List:\n";
    for (size_t i(0); i<dl.size(); ++i)
      std::cerr << (boost::format("(%2d) '%s'") % i % dl[i]->name()) << std::endl;
    throw std::runtime_error(str(boost::format("Audio Device %s not found.") % device_name));
  } else
    return dl[0];
}

typedef struct iq {
  float i;
  float q;
} iq;

class pa_fft : public portaudio::stream_callback::process_base {
public:
  typedef boost::shared_ptr<pa_fft> sptr;
  typedef float FFTType; 
  typedef std::vector<std::complex<FFTType> > fft_vector;

  pa_fft(double center_frequency,
         double sample_rate,
         size_t frames,
         size_t numAverage)
    : center_frequency_(center_frequency)
    , sample_rate_(sample_rate)
    , frames_(frames)
    , fft_(frames, FFTW_FORWARD, FFTW_ESTIMATE)
    , ps_(center_frequency_-47e3, center_frequency_+47e3, sample_rate_, 0)
    , ps_sum_(ps_)
    , counter_(0)
    , numAverage_(numAverage) {}

  virtual ~pa_fft() {}

  static sptr make(double center_frequency,
                   double sample_rate,
                   size_t frames,
                   size_t numAverage) {    
    return sptr(new pa_fft(center_frequency, sample_rate, frames, numAverage));
  }

  virtual void init() {
    counter_    = 0;
  }
  virtual int process(const void *input_buffer, 
                      void *output_buffer,
                      unsigned long frames_per_buffer,
                      portaudio::callback_info::sptr info) {
    std::cout << "pa_fft callback called " << frames_per_buffer << " " << info->to_string() 
              << " " << info->input_buffer_adc_time() - info->current_time() << std::endl;
    fft_vector::const_iterator begin(static_cast<const fft_vector::value_type*>(input_buffer));
    fft_vector::const_iterator end(begin+frames_per_buffer);
    fft_.transformRange(begin, end, FFT::WindowFunction::Blackman<FFTType>());
    FFTWSpectrum<FFTType> fs(fft_, sample_rate_, center_frequency_);
    ps_.fill(fs, std::abs<double>);
    if (counter_++==0) 
      ps_sum_ = ps_;
    else 
      ps_sum_ += ps_;
    if (counter_ == numAverage_) {
      ps_sum_ /= numAverage_;
    }
    std::cout << "#c = " << counter_ << std::endl;
    return (counter_ == numAverage_) ? paAbort : paContinue;
  }
protected:
private:
  const double center_frequency_;
  const double sample_rate_;
  const size_t frames_;
  FFT::FFTWTransform<FFTType> fft_;
  frequency_vector<FFTType> ps_;
  frequency_vector<FFTType> ps_sum_;
  size_t counter_;
  const size_t numAverage_;
} ;

int main(int argc, char* argv[])
{
  try {
    std::string filename((argc > 1 ) ? argv[1] : "config.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config, boost::property_tree::xml_parser::no_comments);
    const boost::property_tree::ptree& config_fifi_daq(config.get_child("FiFiDAQ.FiFi-SDR"));
    const boost::property_tree::ptree& config_actions(config.get_child("FiFiDAQ.Actions"));

    portaudio::init::sptr pa(portaudio::init::make());

    usb_device_handle::sptr      ud(find_usb_device(config_fifi_daq));
    portaudio::device_info::sptr ad(find_portaudio_device(config_fifi_daq, pa));    

    FiFiSDR::receiver_control::sptr rec(FiFiSDR::receiver_control::make(ud));

    portaudio::stream_parameters::sptr 
      input_parameters(portaudio::stream_parameters::make(ad->index(),
                                                          2,
                                                          paFloat32,
                                                          ad->default_high_input_latency()));
    portaudio::stream_parameters::sptr output_parameters;
    const double sample_rate(config_fifi_daq.get<double>("<xmlattr>.sampleRate_Hz"));

    if (pa->is_format_supported(input_parameters,
                                output_parameters,
                                sample_rate)) {
#if 0      
      std::vector<std::complex<float> > buffer(frames);
      portaudio::stream_blocking::sptr
        sb(portaudio::stream_blocking::make(input_parameters,
                                            output_parameters,
                                            sample_rate,
                                            frames,
                                            paNoFlag));
      typedef float FFTType; 
      FFT::FFTWTransform<FFTType> fft(frames, FFTW_FORWARD, FFTW_ESTIMATE);
      frequency_vector<FFTType> ps(center_freq-47e3, center_freq+47e3, sample_rate, 0);
      frequency_vector<FFTType> ps_sum(center_freq-47e3, center_freq+47e3, sample_rate, 0);
      sb->start();    
      for (int i=0; i<100; ++i) {        
        try {
          sb->read_data(&buffer[0], frames);          
          fft.transformVector(buffer, FFT::WindowFunction::Blackman<FFTType>());
          FFTWSpectrum<FFTType> fs(fft, sample_rate, center_freq);
          ps.fill(fs, std::abs<double>);
          if (i==0) ps_sum=ps;
          else  ps_sum += ps;
        } catch (const portaudio::input_overflow& err) {
          std::cout << err.what() << std::endl;
        }
      }
      sb->stop();
      for (frequency_vector<FFTType>::const_iterator j(ps_sum.begin()); j!=ps_sum.end(); ++j) {
        std::cout << j->first << " "<< j->second << std::endl;
      }
#endif
    }

    std::map<std::string, pa_fft::sptr> procs;
    BOOST_FOREACH(const boost::property_tree::ptree::value_type& action, config_actions) {
      const std::string action_name(action.second.get<std::string>("<xmlattr>.label"));
      std::cout << "action: " << action.first << " name= "<< action_name << std::endl;
      if (action.first == "FFTPowerSpectrum") {
        // center frequency
        const double_t center_freq(action.second.get<double>("<xmlattr>.centerFrequency_Hz"));
        rec->set_frequency(1e-6*center_freq);
        
        const double df(action.second.get<double>("<xmlattr>.frequencyResolution_Hz"));
        const size_t frames(sample_rate/df);        
        const size_t numAverage(action.second.get<unsigned>("<xmlattr>.numAverage"));
        std::cout << "sampleRate= " << sample_rate << " frames= " << frames << std::endl;
        if (procs.find(action_name) == procs.end()) {
          procs.insert(std::make_pair(action_name, 
                                      pa_fft::make(center_freq, sample_rate, frames, numAverage)));
        }
        portaudio::stream_callback::sptr
          sb(portaudio::stream_callback::make(input_parameters,
                                              output_parameters,
                                              sample_rate,
                                              frames,
                                              paNoFlag, 
                                              procs[action_name]));
        
        procs[action_name]->init();
        sb->start();
        while (sb->is_active())
          pa->sleep(0.05);
        sb->stop();
      } else
        throw std::runtime_error("unknown action " + action.first);
    }

    std::cout << "frequency = " << rec->get_frequency() << std::endl;
    rec->set_frequency(10.0);
    std::cout << "frequency = " << rec->get_frequency() << std::endl;

    std::cout << "xtal frequency = " << rec->get_xtal_frequency() << std::endl;
    rec->set_xtal_frequency(rec->get_xtal_frequency());
    std::cout << "xtal frequency = " << rec->get_xtal_frequency() << std::endl;

    FiFiSDR::receiver_control::presel_entry pe = rec->get_presel_entry(0);
    std::cout << "f1,f2,num_abpf= " << pe.freq1() << " "<< pe.freq2() << " " << rec->num_abpf() << std::endl;

  } catch (const std::runtime_error& e) {
    std::cerr << e.what() << std::endl;
  }
}
