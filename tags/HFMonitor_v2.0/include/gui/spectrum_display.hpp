// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
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
#ifndef _spectrum_display_hpp_cm120516_
#define _spectrum_display_hpp_cm120516_

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Text_Display.H>
#include <FL/Fl_Simple_Counter.H>
#include <FL/Fl_Value_Output.H>

#include <vector>
#include <cmath>

class spectrum_display : public Fl_Double_Window {
public:
  enum {
    sMin,
    sMax,
    fMin,
    fMax,
    fCur
  } ;
  spectrum_display(int x, int y, int w, int h)
    : Fl_Double_Window(x, y, w, h)
    , b0_  (  0, h-20,  50, 20, "Min:")
    , sMin_( 50, h-20,  50, 20, "sMin")
    , b1_  (100, h-20,  50, 20, "Max:")
    , sMax_(150, h-20,  50, 20, "sMax")
    , fMin_(250, h-20,  50, 20, "fMin:")
    , fMax_(350, h-20,  50, 20, "fMax:")
    , fCur_(500, h-20,  70, 20, "fCur:") {
    sMin_.step(5,5); sMin_.range(-120,0); sMin_.value(-120);
    sMax_.step(5,5); sMax_.range(-120,0); sMax_.value(0);

    fMin_.value( 9700); fMax_.value(10100);
    fCur_.value(-1); fCur_.precision(3); fCur_.range(0, 40e3);

    sMin_.callback(&cb, sMin); sMax_.callback(&cb, sMax);
    fMin_.callback(&cb, fMin); fMax_.callback(&cb, fMax);

    end();
    specIndex_ = specM();
    spec_.resize(specN() * specM());
    specImg_.resize(2 * specN() * specM());
  }
  virtual ~spectrum_display() {}

  static void cb(Fl_Widget *w, long u) {
    spectrum_display *s = (spectrum_display*)w->parent();
    const double v = ((Fl_Valuator*)(w))->value();
    switch (u) {
    case sMin: s->sMax_.minimum(v+5);
      break; 
    case sMax: s->sMin_.maximum(v-5);
      break;
    }
    s->damage(FL_DAMAGE_ALL);
  }
  
  void insert_spec(const std::vector<double>& spec) {
    specIndex_--;
    if (specIndex_ <0) specIndex_ = specM()-1;
    for (size_t i=0; i<spec.size(); ++i) {
      spec_[specIndex_*specN()+i] = spec[i];
      double x = (spec[i]-sMin_.value())/(sMax_.value()-sMin_.value());
      x= (x<0) ? 0 : (x>1) ? 1 : x;
      specImg_[specIndex_*specN()+i] = specImg_[(specM()+specIndex_)*specN()+i] = 255*x;
    }
    this->damage(FL_DAMAGE_ALL);
  }
  
  void draw() {
    Fl_Widget *const*a = array();
    if (damage() == FL_DAMAGE_CHILD) { // only redraw some children
      for (int i = children(); i--; a++)
	update_child(**a);
    } else { // total redraw
      fl_draw_box(FL_FLAT_BOX, 0, 0, w(), h(), FL_GRAY);
      draw_spec();
      draw_waterfall();
      draw_frames();
      draw_ticks(50); // step size 50 kHz for now

      // now draw all the children atop the background:
      for (int i = children(); i--; a++) {
	draw_child(**a);
	draw_outside_label(**a); // you may not need to do this
      }
    }
  }

  void draw_spec() const {
    fl_color(FL_GREEN);
    fl_begin_line();
    for (int i=0; i<specN(); ++i)
      fl_vertex(i, ySpecFromInput(spec_[specIndex_*specN() + i]));
    fl_end_line();
  }

  void draw_waterfall() const {
    // Fl_RGB_Image img(&specImg_[0]+specIndex_*specN(), specN(), specM(), 1);
    // img.draw(xWaterfallBeg(), yWaterfallBeg());
    fl_draw_image_mono(&specImg_[0]+specIndex_*specN(),
		       xWaterfallBeg(), yWaterfallBeg(),
		       specN(),         specM());
  }

  void draw_frames() {
    fl_color(FL_BLACK);
    fl_rect(0, ySpecBeg(),      w(), ySpecEnd()-ySpecBeg());
    fl_rect(0, yWaterfallBeg(), w(), yWaterfallEnd()-yWaterfallBeg());
  }

  // every step kHz
  void draw_ticks(int step) {
    fl_color(FL_BLUE);
    char label[1024];
    fl_line_style(FL_DASH);
    for (int fi=step*(int(fMin_.value())/step); fi <= fMax_.value(); fi += step) {
      if (fi < fMin_.value()) continue;
      const int xl = xSpecFromInput(fi);
      sprintf(label, "%d", fi);
      const double labelWidth = fl_width(label);
      if (xl-labelWidth/2 <= xSpecBeg()) continue;
      if (xl+labelWidth/2 >= xSpecEnd()) continue;
      fl_line(xl, ySpecBeg(),      xl, ySpecEnd());
      fl_line(xl, yWaterfallBeg(), xl, yWaterfallEnd());
      fl_draw(label, xl-labelWidth/2, yWaterfallBeg()-fl_descent());
    }
  }

  int handle(int event) {
    switch (event) {
    case FL_MOVE:
      fCur_.value(0);
      if (xSpecBeg() < Fl::event_x() && Fl::event_x() < xSpecEnd()) {
	if ((ySpecBeg()      < Fl::event_y() && Fl::event_y() < ySpecEnd()) ||
	    (yWaterfallBeg() < Fl::event_y() && Fl::event_y() < yWaterfallEnd()))	  
	  fCur_.value(xInputFromSpec(Fl::event_x()));
      }
    default:
      return Fl_Double_Window::handle(event);
    }
  }
protected:
  int ySpecBeg() const { return  0; }
  int ySpecEnd() const { return 100; }
  int yWaterfallBeg() const { return ySpecEnd()+16; }
  int yWaterfallEnd() const { return h()-20; }

  int xSpecBeg() const { return  0; }
  int xSpecEnd() const { return w(); }
  int xWaterfallBeg() const { return 0; }
  int xWaterfallEnd() const { return w(); }

  int specN() const { return xSpecEnd()      - xSpecBeg(); }
  int specM() const { return yWaterfallEnd() - yWaterfallBeg(); }

  int xSpecFromInput(double xi) const {
    double x = (xi-fMin_.value()) / (fMax_.value()-fMin_.value());
    x = (x<0) ? 0. : (x>1) ? 1. : x;
    return xSpecBeg() + x * (xSpecEnd()-xSpecBeg()-1);
  }
  int ySpecFromInput(double yi) const {
    double x = (yi-sMin_.value()) / (sMax_.value()-sMin_.value());
    x = (x<0) ? 0. : (x>1) ? 1. : x;
    return ySpecEnd()-1 - x * (ySpecEnd()-ySpecBeg());
  }
  double xInputFromSpec(int i) const {
    double x = double(i-xSpecBeg())/double(xSpecEnd()-xSpecBeg());
    return (x<0) ? -1 : (x>1) ? -1 : fMin_.value() + x * (fMax_.value()-fMin_.value());
  }
private:
  Fl_Box b0_;
  Fl_Simple_Counter sMin_;
  Fl_Box b1_;
  Fl_Simple_Counter sMax_;
  Fl_Value_Output   fMin_;
  Fl_Value_Output   fMax_;
  Fl_Value_Output   fCur_;
  int                 specIndex_;
  std::vector<double> spec_;
  std::vector<uchar>  specImg_;
} ;

#endif // _spectrum_display_hpp_cm120516_
