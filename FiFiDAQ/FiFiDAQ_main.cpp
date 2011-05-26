// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <fstream>
#include <sstream>
#include <octave/oct.h>
#include <octave/Matrix.h>
#include <octave/ov.h>
#include <octave/load-save.h>
#include <octave/ls-mat5.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
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
  typedef boost::property_tree::ptree ptree;
  typedef boost::posix_time::ptime ptime;

  mat_spectrum_saver(const ptree& config_proc, 
                     const ptree& config_saver)
    : name_(config_proc.get<std::string>("<xmlattr>.label"))
    , byte_limit_(1024*get_opt_def<size_t>(config_proc, config_saver, "<xmlattr>.maxBufferSize_kB", 500))
    , base_path_(get_opt_def<std::string>(config_proc, config_saver, "<xmlattr>.dataPath", "Data/"))
    , minutes_(std::min(size_t(60), 
                        get_opt_def<size_t>(config_proc, config_saver, "<xmlattr>.minutesPerFile", 10)))
    , compress_(false)
    , save_as_floats_(false)
    , mat7_format_(true)
    , global_(false)
    , pos_(0)
    , counter_(0)
    , current_data_size_(0) {}
  
  template<typename fft_type>
  void save_spectrum(const frequency_vector<fft_type>& spec, ptime t) {
    const boost::filesystem::path filepath(gen_filepath(t));
    if (boost::filesystem::exists(filepath) and (pos_ == std::streampos(0))) {
      std::cerr << "file '" << filepath << "' exists and will be overwritten" << std::endl;
      boost::filesystem::remove(filepath);        
    }
    if (not boost::filesystem::exists(filepath)) {
      boost::filesystem::ofstream ofs(filepath, std::ios::binary);
      write_header(ofs, load_save_format(LS_MAT5_BINARY));
      freq_ = Matrix(1, spec.size());
      spec_ = int16NDArray(dim_vector(0, spec.size()));
      time_ = Matrix(0, 1);
      for (size_t i=0; i<spec.size(); ++i)
        freq_.elem(0, i) = spec[i].first;
      counter_ = current_data_size_ = 0 ;
      save_mat_var(ofs, freq_, "freq_Hz");
      pos_ = ofs.tellp(); 
      save_mat_var(ofs, spec_, "spec"+varname_suffix());
      save_mat_var(ofs, time_, "time"+varname_suffix());
    }
#if 0    
    std::string name("");
    std::string vs(varname_suffix());
    bool swap(false);
    octave_value ov;
    // while ((name=read_mat5_binary_element(ifs, "", swap, global_, ov)) !="")  {
    name= read_mat5_binary_element(ifs, "", swap, global_, ov);
    if (name == "spec_"+varname_suffix()) {
      old_spec = ov.int16_array_value(); std::cout << "--- found spec ---" <<std::endl; }
    name= read_mat5_binary_element(ifs, "", swap, global_, ov);
    if (name == "time_"+varname_suffix()) {
      old_time = ov.matrix_value(); std::cout << "--- found time ---" <<std::endl; }
  // }
#endif
    int16NDArray old_spec(spec_);
    Matrix       old_time(time_);
    size_t n(old_spec.rows()); 

    std::cout<< "--- current_data_size_ = " 
             << current_data_size_ << " " 
             << old_spec.byte_size() + old_time.byte_size() << std::endl;
    if (current_data_size_ > byte_limit_) {
      n = 0;
      current_data_size_ = 0;
      counter_++;
      boost::filesystem::ifstream ifs(filepath, std::ios::binary);
      ifs.seekg(0, std::ios::end);
      pos_ = ifs.tellg();
      old_spec.resize(dim_vector(0, spec.size()));
      old_time.resize(0, 1);
    }
    spec_.resize(dim_vector(n+1, spec.size()));
    time_.resize(n+1, 1);

    spec_.insert(old_spec, 0, 0);
    time_.insert(old_time, 0, 0);
    for (size_t i=0; i<spec.size(); ++i)
      spec_.elem(n, i) = boost::int16_t(100 * 20.*std::log10(spec[i].second) + 0.5);
    time_.elem(n, 0) = ptime_to_datenum(t);

    // overwrite old variables with new ones
    boost::filesystem::fstream ofs(filepath, std::ios::binary | std::ios::in | std::ios::out);
    ofs.seekp(pos_);
    save_mat_var(ofs, spec_, "spec_"+varname_suffix());
    save_mat_var(ofs, time_, "time_"+varname_suffix());
    current_data_size_ = ofs.tellp() - pos_;
  }
protected:
  bool save_mat_var(std::ostream& os,
                    const octave_value& ov,
                    std::string varname) const {
    return save_mat5_binary_element(os, ov, varname, global_, mat7_format_, save_as_floats_, compress_);
  }
  boost::filesystem::path gen_filepath(ptime t) const {
    boost::filesystem::path p(base_path_+"/"+name_);
    boost::filesystem::create_directories(p);

    std::stringstream oss;
    oss.imbue(std::locale(oss.getloc(), new boost::posix_time::time_facet("y%Y-m%m-d%d_H%HM%M")));
    const boost::posix_time::time_duration td(t.time_of_day());         
    t= boost::posix_time::ptime(t.date(), 
                                boost::posix_time::time_duration(td.hours(), 
                                                                 minutes_*(td.minutes()/minutes_), 0));
    oss << t << ".mat";
    return p/=(oss.str());
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
  template<typename T>
  static
  T get_opt_def(const ptree& p1, const ptree& p2, std::string name, T def) {
    return p1.get<T>(name, p2.get<T>(name, def));
  }
private:
  const std::string name_;
  const size_t      byte_limit_;
  const std::string base_path_;
  const size_t      minutes_;
  const bool        compress_;
  const bool        save_as_floats_;
  const bool        mat7_format_;
  bool              global_;
  std::streampos    pos_;
  size_t            counter_;
  size_t            current_data_size_;
  Matrix            freq_;
  int16NDArray      spec_;
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

  pa_fft(const ptree& config, 
         const ptree& config_saver, 
         double sample_rate)
    : center_frequency_(config.get<double>("<xmlattr>.centerFrequency_Hz"))
    , sample_rate_(sample_rate)
    , frames_(sample_rate_/config.get<double>("<xmlattr>.frequencyResolution_Hz"))
    , num_average_(config.get<size_t>("<xmlattr>.numAverage"))
    , fft_(frames_, FFTW_FORWARD, FFTW_ESTIMATE)
    , ps_(config.get<double>("<xmlattr>.frequencyMin_Hz", center_frequency_-0.5*47/48*sample_rate_),
          config.get<double>("<xmlattr>.frequencyMax_Hz", center_frequency_+0.5*47/48*sample_rate_), 
          sample_rate_, 0)
    , ps_sum_(ps_)
    , counter_(0)
    , saver_(config, config_saver) {}

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

  static sptr make(const ptree& config, const ptree& config_saver, double sample_rate) {
    return sptr(new pa_fft(config, config_saver, sample_rate));
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
    // std::cout << "pa_fft callback called " << frames_per_buffer << " " << info->to_string() 
    //           << " " << info->input_buffer_adc_time() - info->current_time() << std::endl;
    fft_vector::const_iterator begin(static_cast<const fft_vector::value_type*>(input_buffer));
    fft_vector::const_iterator end(begin+frames_per_buffer);
    fft_.transformRange(begin, end, FFT::WindowFunction::Blackman<FFTType>());
    FFTWSpectrum<FFTType> fs(fft_, sample_rate_, center_frequency_);
    ps_.fill(fs, std::abs<double>);
    if (counter_++ == 0) 
      ps_sum_ = ps_;
    else 
      ps_sum_ += ps_;
    if (counter_ == num_average_) {
      ps_sum_ /= num_average_;
      const ptime now(boost::posix_time::microsec_clock::universal_time());
      const time_duration dt(0, 0, 0,
                             num_average_/2 * frames_per_buffer/sample_rate_*time_duration::ticks_per_second());
      saver_.save_spectrum(ps_sum_, now-dt);
    }
    return (counter_ == num_average_) ? paAbort : paContinue;
  }
protected:
private:
  const double center_frequency_;
  const double sample_rate_;
  const size_t frames_;
  const size_t num_average_;
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
    const boost::property_tree::ptree& config_saver(config.get_child("FiFiDAQ.MatSaver"));
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
            procs.insert(std::make_pair(action_name, pa_fft::make(action.second, config_saver, sample_rate)));
          
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
