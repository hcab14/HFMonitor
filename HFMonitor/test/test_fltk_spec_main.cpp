// -*- C++ -*-
// $Id$

#include <complex>
#include <iostream>
#include <boost/property_tree/xml_parser.hpp>

#include "FFT.hpp"
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
    : Fl_Double_Window(w, h)
    , menu_bar_(0,0,w,20)
    , input_(0, h-30, w, 30, "label")
    , specDisplay_(20, 20, w-40, h-50)
    , counter_(0)
    // , disp_(20, 40, w-40, h-40, "Display") 
  {
    menu_bar_.add("File/Quit",   FL_CTRL+'q', Quit_CB);
    menu_bar_.add("Edit/Change", FL_CTRL+'c', Change_CB);
    menu_bar_.add("Edit/Submenu/Aaa");
    menu_bar_.add("Edit/Submenu/Bbb");
    // box1_.box(FL_UP_BOX);
    // box1_.labelfont(FL_BOLD|FL_ITALIC);
    // box1_.labelsize(18);
    // box1_.labeltype(FL_SHADOW_LABEL);
    // box2_.box(FL_UP_BOX);
    // box1_.labelcolor(0x00FF0000);
    // input_.value("Now is the time for all good men...");
    end();
    // Fl::add_timeout(0.1, (Fl_Timeout_Handler)timeout_cb, (void *)this);
  }
  virtual ~MyWindow() {
    Fl::remove_timeout((Fl_Timeout_Handler)timeout_cb, (void *)this);
  }

  static void timeout_cb(MyWindow *w) {
    std::vector<double> spec;
    for (size_t i=0; i<1000; ++i)
      spec.push_back(-20*(1+sin(0.1*i + w->counter_*0.1)));
    w->get_spec_display().insert_spec(spec);
    w->counter_++;
    Fl::repeat_timeout(0.1, (Fl_Timeout_Handler)timeout_cb, (void *)w);
  }
  spectrum_display& get_spec_display() { return specDisplay_; }

protected:
  static void Change_CB(Fl_Widget *w, void *) {
    // MyWindow *wp = (MyWindow*)(p);
    
    Fl_Menu_Bar *menu = (Fl_Menu_Bar*)w;
    Fl_Menu_Item *p;
    // Change submenu name
    p = (Fl_Menu_Item*)menu->find_item("Edit/Submenu");
    if ( p ) p->label("New Submenu Name");
    // Change item name
    p = (Fl_Menu_Item*)menu->find_item("Edit/New Submenu Name/Aaa");
    if ( p ) p->label("New Aaa Name");
  }
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
  test_proc(const boost::property_tree::ptree&)
    : w_(1200,400)
    , fftw_(1024, FFTW_BACKWARD, FFTW_ESTIMATE) {
    w_.show();
  }

  void process_iq(processor::service_iq::sptr sp,
                  std::vector<std::complex<double> >::const_iterator i0,
                  std::vector<std::complex<double> >::const_iterator i1) {
    const size_t length(std::distance(i0, i1));
    std::cout << "process_iq nS=" << std::distance(i0, i1) 
              << " " << sp->id()
              << " " << sp->approx_ptime()
              << " " << sp->sample_rate_Hz()
              << " " << sp->center_frequency_Hz()
              << " " << sp->offset_ppb()
	      << " length=" << length
              << std::endl;

    if (length != fftw_.size())
      fftw_.resize(length);
    fftw_.transformRange(i0, i1, FFT::WindowFunction::Blackman<double>(length));
    const FFTWSpectrum<double> s(fftw_, sp->sample_rate_Hz(), sp->center_frequency_Hz());
    const double f_min(sp->center_frequency_Hz() - sp->sample_rate_Hz());
    const double f_max(sp->center_frequency_Hz() + sp->sample_rate_Hz());
    const frequency_vector<double> ps(f_min, f_max, s, s2db);
    w_.get_spec_display().insert_spec(ps, sp);
    char msg[1024]; sprintf(msg,"spec_update");
    Fl::awake(msg);
  }
private:
  static double s2db(std::complex<double> c) {
    return 10*std::log10(std::abs(c));
  }
  MyWindow w_;
  FFT::FFTWTransform<double> fftw_;
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
  
  LOGGER_INIT("./Log", "test_fltk_spec");

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
