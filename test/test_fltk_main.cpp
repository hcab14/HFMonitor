// -*- C++ -*-
// $Id$

#include <iostream>
#include <vector>
#include <cmath>

#include <FL/Fl.H>
#include <FL/Fl_Select_Browser.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_Text_Display.H>

#include "gui/spectrum_display.hpp"

#include "db/sqlite3.hpp"

class MainWindow : public Fl_Double_Window {
public:
  MainWindow(int w, int h)
    : Fl_Double_Window(w, h)
    , menu_bar_(0,0,w,20)
    , input_(0, h-30, w, 30, "label")
    , browser_(20, 20, 200, h-50)
    // , disp_(20, 40, w-40, h-40, "Display")
  {
    menu_bar_.add("File/Open",   FL_CTRL+'o', Open_CB, this);
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
  virtual ~MainWindow() {
    Fl::remove_timeout((Fl_Timeout_Handler)timeout_cb, (void *)this);
  }

  static void timeout_cb(MainWindow *w) {
    // std::vector<double> spec;
    // for (size_t i=0; i<1000; ++i)
    //   spec.push_back(-20*(1+sin(0.1*i + w->counter_*0.1)));
    // w->get_spec_display().insert_spec(spec, spec);
    // w->counter_++;
    Fl::repeat_timeout(0.1, (Fl_Timeout_Handler)timeout_cb, (void *)w);
  }

protected:
  void message() const {
    std::cout << "Hi\n";
  }
  static void Change_CB(Fl_Widget *w, void *) {
    // MainWindow *wp = (MainWindow*)(p);

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
  static void Open_CB(Fl_Widget *w, void *p) {
    std::cout << "open " << w << " " << p << std::endl;
    Fl_File_Chooser fc("./", "*.db", Fl_File_Chooser::SINGLE, "title");
    fc.show();
    while (fc.visible())
      Fl::wait();

    char relative[FL_PATH_MAX];
    const int count = fc.count();
    if (!count)
      return;
    fl_filename_relative(relative, sizeof(relative), fc.value(0));
    std::cout << "selected file: " << relative << std::endl;

    MainWindow *mainWindow = reinterpret_cast<MainWindow*>(p);
    mainWindow->open_db(std::string(relative));
  }
  void open_db(std::string file_name) {
    connection_ = db::sqlite3::connection::make(str(boost::format("file:%s?mode=rwc") % file_name));
    update_table();
  }
  void update_table() {
    if (!connection_)
      return;
    db::sqlite3::statement s(connection_, "SELECT freqId,f FROM Freq;");
    while (s.step()) {
      const boost::int64_t freqId = s.get_column<boost::int64_t>(0);
      const float          f      = s.get_column<float>(1);
      std::string txt = str(boost::format("%.0f") % f);
      std::cout << "F: " << freqId << " " << txt << std::endl;
      // //      browser_.data(freqId, (void*)txt.c_str());
      // static char data[100] = "data";
      browser_.add(txt.c_str());
    }
    browser_.redraw();
  }

private:
  Fl_Menu_Bar menu_bar_;
  Fl_Input    input_;
  Fl_Select_Browser  browser_;
  // Fl_Text_Display disp_;
  // Fl_Text_Buffer buff_;
  db::sqlite3::connection::sptr connection_;
} ;

int main(int argc, char **argv) {
  MainWindow w(1200,600);
  w.show(argc,argv);
  return Fl::run();
}
