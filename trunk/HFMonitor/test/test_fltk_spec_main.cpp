// -*- C++ -*-
// $Id$

#include <iostream>
#include <boost/property_tree/xml_parser.hpp>

#include "gui/spectrum_display.hpp"
#include "network.hpp"
#include "network/client.hpp"
#include "network/iq_adapter.hpp"
#include "repack_processor.hpp"
#include "run.hpp"

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
    input_.value("Now is the time for all good men...");
    end();
    Fl::add_timeout(0.1, (Fl_Timeout_Handler)timeout_cb, (void *)this);
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
    Fl::repeat_timeout(0., (Fl_Timeout_Handler)timeout_cb, (void *)w);
  }

protected:
  spectrum_display& get_spec_display() { return specDisplay_; }
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
    exit(0);
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
  test_proc(const boost::property_tree::ptree&) {}

  void process_iq(processor::service_iq::sptr sp,
                  std::vector<std::complex<double> >::const_iterator i0,
                  std::vector<std::complex<double> >::const_iterator i1) {
    std::cout << "process_iq nS=" << std::distance(i0, i1) 
              << " " << sp->id()
              << " " << sp->approx_ptime()
              << " " << sp->sample_rate_Hz()
              << " " << sp->center_frequency_Hz()
              << " " << sp->offset_ppb()
              << std::endl;
  }
} ;

struct main_func {
  main_func(int argc, char **argv)
    : argc_(argc)
    , argv_(argv) {}

  void operator()() {
    
  }
  int    argc_;
  char** argv_;
} ;

void* main_func(int argc, char **argv, void* p) {
  MyWindow* w = (MyWindow *)(p);

  LOGGER_INIT("./Log", "test_client_fltk");
  try {
    const std::string filename((argc > 1 ) ? argv[1] : "config_client.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config);

    const std::string stream_name("DataIQ");

    client<iq_adapter<repack_processor<test_proc> > >
      c(config.get_child("Server"));
    const std::set<std::string> streams(c.ls());
    if (streams.find(stream_name) != streams.end())
      ASSERT_THROW(c.connect_to(stream_name) == true);
    else
      throw std::runtime_error(str(boost::format("stream '%s' is not available")
                                   % stream_name));
    c.start();
    run_in_thread(network::get_io_service());
  } catch (const std::exception &e) {
    LOG_ERROR(e.what());
  }
  return NULL;
}

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_client");
  try {
    const std::string filename((argc > 1 ) ? argv[1] : "config_client.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config);

    const std::string stream_name("DataIQ");

    client<iq_adapter<repack_processor<test_proc> > >
      c(config.get_child("Server"));
    const std::set<std::string> streams(c.ls());
    if (streams.find(stream_name) != streams.end())
      ASSERT_THROW(c.connect_to(stream_name) == true);
    else
      throw std::runtime_error(str(boost::format("stream '%s' is not available")
                                   % stream_name));
    c.start();

    typedef boost::shared_ptr<boost::thread> thread_sptr;
    thread_sptr tp(new boost::thread(boost::bind(&boost::asio::io_service::run,
						 &network::get_io_service())));

    // wait Fl:...

  

  io_service.stop();
  tp->join();

  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
