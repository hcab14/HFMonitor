// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <fstream>
#include <octave/oct.h>
#include <octave/Matrix.h>
#include <octave/ov.h>
#include <octave/load-save.h>
#include <octave/ls-mat5.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
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
    std::cerr << "Available USB devices:\n";
    const std::vector<usb_device_handle::sptr> dl(usb_device_handle::get_device_list(0, 0));
    BOOST_FOREACH(usb_device_handle::sptr device, dl) {
      std::cerr << (boost::format("vendor_id=%5d product_id=%5d serial='%s'") 
                    % device->get_vendor_id() 
                    % device->get_product_id()
                    % device->get_serial()) << std::endl;
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
    std::cerr << "Audio Device List:\n";
    const portaudio::init::device_list dl(pa->get_device_list());
    for (size_t i(0); i<dl.size(); ++i)
      std::cerr << (boost::format("(%2d) '%s'") % i % dl[i]->name()) << std::endl;
    throw std::runtime_error(str(boost::format("Audio Device %s not found.") % device_name));
  } else
    return dl[0];
}

class mat_spectrum_saver {
public:
  typedef boost::posix_time::ptime ptime;
  mat_spectrum_saver(std::string name)
    : name_(name)
    , compress_(false)
    , save_as_floats_(false)
    , global_(false)
    , pos_(0)
    , counter_(0) {}

  template<typename fft_type>
  void save_spectrum(const frequency_vector<fft_type>& spec, ptime t) {
    const std::string filename(gen_filename(t));
    Matrix old_spec;
    Matrix old_time;
    if (not boost::filesystem::exists(filename)) {
      std::ofstream ofs(filename.c_str(), std::ios::binary);
      write_header(ofs, load_save_format(LS_MAT5_BINARY));
      freq_.resize(1, spec.size());
      spec_.resize(0, spec.size());
      time_.resize(0, 1);
      for (size_t i=0; i<spec.size(); ++i)
        freq_.elem(0, i) = spec[i].first;
      counter_ = 0;
      save_mat5_binary_element(ofs, freq_, "freq_Hz",               global_, false, save_as_floats_, compress_);
      pos_ = ofs.tellp(); 
      save_mat5_binary_element(ofs, spec_, "spec"+varname_suffix(), global_, false, save_as_floats_, compress_);
      save_mat5_binary_element(ofs, time_, "time"+varname_suffix(), global_, false, save_as_floats_, compress_);
    } else {
      // std::ofstream ifs(filename.c_str());
      // bool swap(true);
      // read_mat5_binary_file_header(ifs, swap, true, filename);      
    }
    
    std::fstream ifs(filename.c_str(), std::ios::binary | std::ios::in | std::ios::out);
    ifs.seekg(pos_);
    std::string name("");
    std::string vs(varname_suffix());
    bool swap(false);
    octave_value ov;
    // while ((name=read_mat5_binary_element(ifs, "", swap, global_, ov)) !="")  {
    name=read_mat5_binary_element(ifs, "", swap, global_, ov);
    if (name == "spec"+varname_suffix()) {
      old_spec = ov.matrix_value(); std::cout << "--- found spec ---" <<std::endl; }
    name=read_mat5_binary_element(ifs, "", swap, global_, ov);
    if (name == "time"+varname_suffix()) {
      old_time = ov.matrix_value(); std::cout << "--- found time ---" <<std::endl; }
  // }

    size_t n = old_spec.rows(); 

    if (n > 2) {
      n    = 0;
      counter_++;
      ifs.seekg(0, std::ios::end);
      pos_ = ifs.tellg(); std::cout << "--- new pos "  << pos_ << " ---" << std::endl;
      pos_ = ifs.tellp(); std::cout << "--- new pos "  << pos_ << " ---" << std::endl;
      old_spec = Matrix(0, spec.size());
      old_time = Matrix(0, 1);
    }
    Matrix ospec(n+1, spec.size());
    Matrix otime(n+1, 1);

    ospec.insert(old_spec, 0,0);
    otime.insert(old_time, 0,0);
    for (size_t i=0; i<spec.size(); ++i)
      ospec.elem(n, i) = 20.*std::log10(spec[i].second);
    otime(n, 0) = ptime_to_datenum(t);

    std::cout << "--- spec n,m = " << ospec.rows() << "," << ospec.cols() << " ---" << std::endl;
    std::cout << "--- time n,m = " << otime.rows() << "," << otime.cols() << " ---" << std::endl;

    // overwrite old variables with new ones
    std::fstream ofs(filename.c_str(), std::ios::binary | std::ios::in | std::ios::out);
    ofs.seekp(pos_);
    save_mat5_binary_element(ofs, ospec, "spec"+varname_suffix(), global_, false, save_as_floats_, compress_);
    save_mat5_binary_element(ofs, otime, "time"+varname_suffix(), global_, false, save_as_floats_, compress_);        
    // save_mat5_binary_element(fs, ospec, "spec"+varname_suffix(), global_, false, save_as_floats_, compress_);
    // save_mat5_binary_element(fs, otime, "time"+varname_suffix(), global_, false, save_as_floats_, compress_);        
  }
protected:
  std::string gen_filename(ptime t) const {
    return name_+".mat";
  }
  std::string varname_suffix() const {
    return str(boost::format("%04X") % counter_);
  }
  static double ptime_to_datenum(const ptime& p) {
    using namespace boost::gregorian;
    using namespace boost::posix_time;;
    const time_duration dt(p - ptime(date(1970, Jan, 1)));
    return 719529.+double(dt.ticks())/(60.*60.*24.*time_duration::ticks_per_second());
  }
private:
  const std::string name_;
  const bool        compress_;
  const bool        save_as_floats_;
  bool              global_;
  std::streampos    pos_;
  size_t            counter_;
  Matrix            freq_;
  Matrix            spec_;
  Matrix            time_;  
} ;

class pa_fft : public portaudio::stream_callback::process_base {
public:
  typedef boost::shared_ptr<pa_fft> sptr;
  typedef float FFTType; 
  typedef std::vector<std::complex<FFTType> > fft_vector;
  typedef boost::property_tree::ptree ptree;

  typedef boost::posix_time::ptime ptime;
  typedef boost::posix_time::time_duration time_duration;

  pa_fft(const ptree& config, double sample_rate)
    : center_frequency_(config.get<double>("<xmlattr>.centerFrequency_Hz"))
    , sample_rate_(sample_rate)
    , frames_(sample_rate_/config.get<double>("<xmlattr>.frequencyResolution_Hz"))
    , numAverage_(config.get<size_t>("<xmlattr>.numAverage"))
    , fft_(frames_, FFTW_FORWARD, FFTW_ESTIMATE)
    , ps_(center_frequency_-47e3, center_frequency_+47e3, sample_rate_, 0)
    , ps_sum_(ps_)
    , counter_(0)
    , saver_(config.get<std::string>("<xmlattr>.label")) {}

  virtual ~pa_fft() {}

  struct oct_values {
    oct_values(size_t cols)
      : freq(0,cols)
      , spec(0,cols)
      , time(0,1) {}
    Matrix freq;
    Matrix spec;
    Matrix time;
  } ;

  static sptr make(const ptree& config, double sample_rate) {
    return sptr(new pa_fft(config, sample_rate));
  }

  double center_frequency() const { return center_frequency_; }
  size_t frames() const { return frames_; }

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
      const ptime now(boost::posix_time::microsec_clock::universal_time());
      saver_.save_spectrum(ps_sum_, now);
    }
    std::cout << "#c = " << counter_ << std::endl;
    return (counter_ == numAverage_) ? paAbort : paContinue;
  }
protected:
private:
  const double center_frequency_;
  const double sample_rate_;
  const size_t frames_;
  const size_t numAverage_;
  FFT::FFTWTransform<FFTType> fft_;
  frequency_vector<FFTType> ps_;
  frequency_vector<FFTType> ps_sum_;
  size_t             counter_;
  mat_spectrum_saver saver_;
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

    if (not pa->is_format_supported(input_parameters, output_parameters, sample_rate))
      throw std::runtime_error("audio format is not supported");

    std::map<std::string, pa_fft::sptr> procs;
    while (1) {
      BOOST_FOREACH(const boost::property_tree::ptree::value_type& action, config_actions) {
        const std::string action_name(action.second.get<std::string>("<xmlattr>.label"));
        std::cout << "action: " << action.first << " name= "<< action_name << std::endl;
        if (action.first == "FFTPowerSpectrum") {
          if (procs.find(action_name) == procs.end())
            procs.insert(std::make_pair(action_name, pa_fft::make(action.second, sample_rate)));
          
          portaudio::stream_callback::sptr
            sb(portaudio::stream_callback::make(input_parameters,
                                                output_parameters,
                                                sample_rate,
                                                procs[action_name]->frames(),
                                                paNoFlag, 
                                                procs[action_name]));
          rec->set_frequency(1e-6*procs[action_name]->center_frequency());
          procs[action_name]->init();
          sb->start();
          while (sb->is_active())
            pa->sleep(0.05);
          sb->stop();
        } else
          throw std::runtime_error("unknown action " + action.first);
      }
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
