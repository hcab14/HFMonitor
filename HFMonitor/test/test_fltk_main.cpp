// -*- C++ -*-
// $Id$

#include "gui/spectrum_display.hpp"

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Text_Display.H>

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
    w->get_spec_display().insert_spec(spec, spec);
    w->counter_++;
    Fl::repeat_timeout(0.1, (Fl_Timeout_Handler)timeout_cb, (void *)w);
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

int main(int argc, char **argv) {
  MyWindow w(1200,400);
  w.show(argc,argv);
  return Fl::run();
}
