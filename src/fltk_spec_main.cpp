// -*- C++ -*-
// $Id: fltk_spec_main.cpp 504 2016-04-06 10:45:38Z cmayer $
//
// Copyright 2010-2014 Christoph Mayer
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

#include <complex>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <stdlib.h>

#include "FFT.hpp"
#include "FFTProcessor/Filter.hpp"
#include "gui/spectrum_display.hpp"
#include "network.hpp"
#include "network/client.hpp"
#include "network/iq_adapter.hpp"
#include "processor.hpp"
#include "repack_processor.hpp"
#include "run.hpp"
#include "Spectrum.hpp"

#include "filter/iir.hpp"

#include <FL/Fl.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Check_Button.H>
#include <FL/Fl_Float_Input.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_Text_Display.H>

#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <iostream>
#include <vector>
#include <cmath>

/*! \addtogroup executables
 *  @{
 * \addtogroup fltk_spec fltk_spec
 * fltk_spec
 * - GUI spectrum display
 * - reads an I/Q data stream (TPC/IP) and displays the FFT spectrum
 * - configuration: command-line
 * 
 * @{
 */

/// FLTK window holding the spectrum display
class MyWindow : public Fl_Double_Window {
public:
  MyWindow(int w, int h)
    : Fl_Double_Window(w, h, "Spectrum Display")
    , menu_bar_(0,0,w,20)
    , specDisplay_(20, 20, w-40, h-50)
    , input_        (w-200, h-30, 200, 30, "label")
    , filter_switch_   ( 0, h-30,  70, 30, "filter")
    , filter_alpha_    (120, h-30, 40, 30, "alpha")
    , filter_threshold_(210, h-30, 40, 30, "thr")
    // , disp_(20, 40, w-40, h-40, "Display") 
  {
    menu_bar_.add("File/Quit", FL_CTRL+'q', Quit_CB);
    end();
    filter_alpha_.value("0.4");
    filter_threshold_.value("8");
    this->callback(Quit_CB);
  }
  virtual ~MyWindow() {}

  void set_fMin(double fMin) { specDisplay_.set_fMin(1e3*fMin); }
  void set_fMax(double fMax) { specDisplay_.set_fMax(1e3*fMax); }

  void set_sMin(double sMin) { specDisplay_.set_sMin(sMin); }
  void set_sMax(double sMax) { specDisplay_.set_sMax(sMax); }

  bool   filter_on()        const { return filter_switch_.value() == 1; }
  double filter_alpha()     const { return atof(filter_alpha_.value()); }
  double filter_threshold() const { return atof(filter_threshold_.value()); }

  spectrum_display& get_spec_display() { return specDisplay_; }

protected:
  static void Quit_CB(Fl_Widget *, void *) {
    Fl::lock();
    static char msg[1024];
    sprintf(msg, "quit");
    Fl::awake(msg);
    Fl::unlock();
  }
private:
  Fl_Menu_Bar menu_bar_;
  spectrum_display specDisplay_;
  Fl_Input    input_;
  Fl_Check_Button filter_switch_;
  Fl_Float_Input  filter_alpha_;
  Fl_Float_Input  filter_threshold_;
  // Fl_Text_Display disp_;
  // Fl_Text_Buffer buff_;
} ;

/// processor for computing FFT and inserting the data into @ref MyWindow
class test_proc : public processor::base_iq {
public:
#ifdef USE_CUDA      
  typedef FFT::CUFFTTransform fft_type;
#else
  typedef FFT::FFTWTransform<float> fft_type;
#endif

  typedef boost::posix_time::ptime ptime;

  test_proc(const boost::property_tree::ptree& config)
    : base_iq(config)
    , w_(1200,400)
    , fftw_(1024, FFTW_BACKWARD, FFTW_ESTIMATE)
    , host_(config.get<std::string>("server.<xmlattr>.host"))
    , port_(config.get<std::string>("server.<xmlattr>.port"))
    , last_update_time_(boost::date_time::not_a_date_time)
    , filter_nb_(0.01, 500e3)
    , s_last_(0.01) {
    w_.show();
    w_.set_fMin(config.get<double>("<xmlattr>.fMin_kHz"));
    w_.set_fMax(config.get<double>("<xmlattr>.fMax_kHz"));
    w_.set_sMin(config.get<double>("<xmlattr>.sMin_dB"));
    w_.set_sMax(config.get<double>("<xmlattr>.sMax_dB"));
  }

  void process_iq(processor::service_iq::sptr sp, const_iterator i0, const_iterator i1) {
    const size_t length(std::distance(i0, i1));
#if 0
    std::cout << "process_iq nS=" << std::distance(i0, i1) 
              << " " << sp->id()
              << " " << sp->approx_ptime()
              << " " << sp->sample_rate_Hz()
              << " " << sp->center_frequency_Hz()
              << " " << sp->offset_ppb()
	      << " length=" << length
	      << " " << filter_nb_.get()
              << std::endl;
#endif
    const bool isFirst = (length != fftw_.size());
    if (isFirst) {
      fftw_.resize(length);
      filter_nb_.init(0.05, sp->sample_rate_Hz());
      s_last_ = 0.1;
    }
    if (filter_.empty()) {
      filter_.add(Filter::LowPass<frequency_vector<float> >::make(length/double(sp->sample_rate_Hz())/4., 30.0));
    }

    std::vector<std::complex<double> > sf(length);
    std::vector<std::complex<double> >::iterator is=sf.begin();
    const float filter_alpha     = 1./(1.+sp->sample_rate_Hz() * w_.filter_alpha());
    const bool  filter_on        = w_.filter_on();
    const float filter_threshold = w_.filter_threshold();
    std::cout << "filter " << filter_on << " " << filter_alpha << " " << w_.filter_alpha() << " "  << filter_threshold << std::endl;
    for (const_iterator i=i0; i!=i1; ++i, ++is) {
      const bool b = abs(*i) < filter_threshold*s_last_;
      s_last_ = (1.-filter_alpha)*s_last_ + b*filter_alpha*std::abs(*i) + (1-b)*filter_alpha*s_last_;
      *is = ((b || !filter_on) ? *i  : std::complex<float>(0));
    } 

    fftw_.transformRange(sf.begin(), sf.end(), FFT::WindowFunction::Blackman<double>(length));
    const FFTSpectrum<fft_type> s(fftw_, sp->sample_rate_Hz(), sp->center_frequency_Hz());
    const double f_min(sp->center_frequency_Hz() - sp->sample_rate_Hz());
    const double f_max(sp->center_frequency_Hz() + sp->sample_rate_Hz());
    frequency_vector<float> ps(f_min, f_max, s, std::abs<float>, sp->offset_ppb());
    if (filter_.x().empty())
      filter_.init(sp->approx_ptime(), ps);
    else
      filter_.update(sp->approx_ptime(), ps);

    frequency_vector<float> xf(filter_.x());

    Fl::lock();
    const std::string window_title(str(boost::format("%s @%s:%s") % sp->stream_name() % host_ % port_));
    w_.label(window_title.c_str());
    const bool update_window(last_update_time_ == boost::date_time::not_a_date_time ||
			     sp->approx_ptime() - last_update_time_ > boost::posix_time::time_duration(0,0,2));
    w_.get_spec_display().insert_spec(ps.apply(s2db()), xf.apply(s2db()), sp, update_window);
    if (update_window)
      last_update_time_ = sp->approx_ptime();
    static char msg[1024];
    sprintf(msg,"spec_update");
    Fl::awake(msg);
    Fl::unlock();
  }
private:
  struct s2db {
    double operator()(double c) const {
      return 10*std::log10(c);
    }
  } ;
  MyWindow w_;
  fft_type fftw_;
  Filter::Cascaded<frequency_vector<float> > filter_;
  std::string host_;
  std::string port_;
  boost::posix_time::ptime last_update_time_;
//   boost::mutex mutex_;
  filter::iir_lowpass_1pole<double, double> filter_nb_;
  float s_last_;
} ;

int main(int argc, char* argv[])
{
  // set up command-line options
  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,?",    "produce help message")
    ("version,v", "display version")
    ("host,h",     po::value<std::string>()->default_value("127.0.0.1"), "server hostname")
    ("port,p",     po::value<std::string>()->default_value("18001"),     "server port")
    ("stream,s",   po::value<std::string>()->default_value("DataIQ"),    "stream name")
    ("overlap,o",  po::value<double>()->default_value(   0.),  "buffer overlap (%)")
    ("delta_t,dt", po::value<double>()->default_value(   1.),  "dt (sec)")
    ("fMin_kHz",   po::value<double>()->default_value(   0),   "fMin (kHz)")
    ("fMax_kHz",   po::value<double>()->default_value(  40e3), "fMax (kHz)")
    ("sMin_dB",    po::value<double>()->default_value( -70),  "sMin (dB)")
    ("sMax_dB",    po::value<double>()->default_value( -40),  "sMax (dB)");

  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 1;
    }
    if (vm.count("version")) {
      std::cout << SVN_VERSION_STRING << std::endl;
      return 1;
    }
  } catch (const std::exception &e) {
    std::cout << e.what() << std::endl;
    std::cout << desc << std::endl;
    return 1;
  }
  
  // logger initialization
  LOGGER_INIT("./Log", "fltk_spec");

  try {
    // make up ptree config
    boost::property_tree::ptree config;
    config.put("server.<xmlattr>.host", vm["host"].as<std::string>());
    config.put("server.<xmlattr>.port", vm["port"].as<std::string>());
    config.put("Repack.<xmlattr>.bufferLength_sec", vm["delta_t"].as<double>());
    config.put("Repack.<xmlattr>.overlap_percent", vm["overlap"].as<double>());
    config.put("<xmlattr>.fMin_kHz", vm["fMin_kHz"].as<double>());
    config.put("<xmlattr>.fMax_kHz", vm["fMax_kHz"].as<double>());
    config.put("<xmlattr>.sMin_dB", vm["sMin_dB"].as<double>());
    config.put("<xmlattr>.sMax_dB", vm["sMax_dB"].as<double>());

    // FLTK initialization
    Fl::visual(FL_RGB);
    Fl::lock();

    // connect to server
    const std::string stream_name(vm["stream"].as<std::string>());
    network::client::client<network::iq_adapter<repack_processor<test_proc> > > c(config);

    const std::set<std::string> streams(c.ls());
    if (streams.find(stream_name) != streams.end())
      ASSERT_THROW(c.connect_to(stream_name) == true);
    else
      throw std::runtime_error(str(boost::format("stream '%s' is not available")
                                   % stream_name));
    // start client
    c.start();

    // run io_service in a thread
    boost::asio::io_service& io_service(network::get_io_service());
    typedef boost::shared_ptr<boost::thread> thread_sptr;
    thread_sptr tp(new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service)));

    // FLTK event loop
    while (Fl::wait() > 0) {
      const char* msg(static_cast<const char *>(Fl::thread_message()));
      if (msg) {
	if (std::string(msg) == "quit") break;
      }
    }
    c.stop();
//     io_service.stop();
//     tp->detach();
    tp->timed_join(boost::posix_time::seconds(2));
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
/*! @} 
 *  @} 
 */
