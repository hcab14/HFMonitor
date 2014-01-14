// -*- C++ -*-
// $Id$
//
// Copyright 2010-2013 Christoph Mayer
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

#include "FFT.hpp"
#include "FFTProcessor/Filter.hpp"
#include "gui/spectrum_display.hpp"
#include "network.hpp"
#include "network/client.hpp"
#include "network/iq_adapter.hpp"
#include "repack_processor.hpp"
#include "run.hpp"
#include "Spectrum.hpp"

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Text_Display.H>

#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <iostream>
#include <vector>
#include <cmath>

// global state:
struct global_state {
  static bool run;
};

bool global_state::run = true;

class MyWindow : public Fl_Double_Window {
public:
  MyWindow(int w, int h)
    : Fl_Double_Window(w, h, "Spectrum Display")
    , menu_bar_(0,0,w,20)
    , input_(0, h-30, w, 30, "label")
    , specDisplay_(20, 20, w-40, h-50)
    , counter_(0)
    // , disp_(20, 40, w-40, h-40, "Display") 
  {
    menu_bar_.add("File/Quit",   FL_CTRL+'q', Quit_CB);
    end();
  }
  virtual ~MyWindow() {}

  spectrum_display& get_spec_display() { return specDisplay_; }

protected:
  static void Quit_CB(Fl_Widget *, void *) {
    global_state::run = false;
    char* msg="quit";
    Fl::awake(msg);
  }
private:
  Fl_Menu_Bar menu_bar_;
  Fl_Input    input_;
  spectrum_display specDisplay_;
  int         counter_;
  // Fl_Text_Display disp_;
  // Fl_Text_Buffer buff_;
} ;

class test_proc {
public:
  test_proc(const boost::property_tree::ptree& config)
    : w_(1200,400)
    , fftw_(1024, FFTW_BACKWARD, FFTW_ESTIMATE)
    , host_(config.get<std::string>("server.<xmlattr>.host"))
    , port_(config.get<std::string>("server.<xmlattr>.port")) {
    filter_.add(Filter::LowPass<frequency_vector<double> >::make(1.0, 15));
    w_.show();
  }

  void process_iq(processor::service_iq::sptr sp,
                  std::vector<std::complex<double> >::const_iterator i0,
                  std::vector<std::complex<double> >::const_iterator i1) {
    const size_t length(std::distance(i0, i1));
#if 0
    std::cout << "process_iq nS=" << std::distance(i0, i1) 
              << " " << sp->id()
              << " " << sp->approx_ptime()
              << " " << sp->sample_rate_Hz()
              << " " << sp->center_frequency_Hz()
              << " " << sp->offset_ppb()
	      << " length=" << length
              << std::endl;
#endif
    if (length != fftw_.size())
      fftw_.resize(length);
    fftw_.transformRange(i0, i1, FFT::WindowFunction::Blackman<double>(length));
    const FFTWSpectrum<double> s(fftw_, sp->sample_rate_Hz(), sp->center_frequency_Hz());
    const double f_min(sp->center_frequency_Hz() - sp->sample_rate_Hz());
    const double f_max(sp->center_frequency_Hz() + sp->sample_rate_Hz());
    frequency_vector<double> ps(f_min, f_max, s, std::abs<double>);
    if (filter_.x().empty())
      filter_.init(sp->approx_ptime(), ps);
    else
      filter_.update(sp->approx_ptime(), ps);

    frequency_vector<double> xf(filter_.x());
    const std::string window_title(str(boost::format("%s @%s:%s") % sp->stream_name() % host_ % port_));
    w_.label(window_title.c_str());
    w_.get_spec_display().insert_spec(ps.apply(s2db()), xf.apply(s2db()), sp);
    char msg[1024]; sprintf(msg,"spec_update");
    Fl::awake(msg);
  }
private:
  struct s2db {
    double operator()(double c) const {
      return 10*std::log10(c);
    }
  } ;
  MyWindow w_;
  FFT::FFTWTransform<double> fftw_;
  Filter::Cascaded<frequency_vector<double> > filter_;
  std::string host_;
  std::string port_;
} ;

int main(int argc, char* argv[])
{
  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,?",                                                       "produce help message")
    ("version,v",                                                    "display version")
    ("host,h", po::value<std::string>()->default_value("127.0.0.1"), "server hostname")
    ("port,p", po::value<std::string>()->default_value("18001"),     "server port")
    ("stream,s", po::value<std::string>()->default_value("DataIQ"),  "stream name");

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
  
  LOGGER_INIT("./Log", "fltk_spec");

  try {
    // make up ptree config
    boost::property_tree::ptree config;
    config.put("server.<xmlattr>.host", vm["host"].as<std::string>());
    config.put("server.<xmlattr>.port", vm["port"].as<std::string>());
    config.put("Repack.<xmlattr>.bufferLength_sec", 1);
    config.put("Repack.<xmlattr>.overlap_percent", 0);

    Fl::visual(FL_RGB);
    Fl::lock();

    const std::string stream_name(vm["stream"].as<std::string>());
    client<iq_adapter<repack_processor<test_proc> > > c(config);

    const std::set<std::string> streams(c.ls());
    if (streams.find(stream_name) != streams.end())
      ASSERT_THROW(c.connect_to(stream_name) == true);
    else
      throw std::runtime_error(str(boost::format("stream '%s' is not available")
                                   % stream_name));
    c.start();

    // run io_service in a thread
    boost::asio::io_service& io_service(network::get_io_service());
    typedef boost::shared_ptr<boost::thread> thread_sptr;
    thread_sptr tp(new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service)));


    // FLTK event loop
    //    Fl::run();
    while (Fl::wait() > 0) {
      if (Fl::thread_message())
	if (!global_state::run) break;
    }

    // now all FLTK windows are closed:
    io_service.stop();
    tp->join();

  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
