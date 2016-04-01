// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
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
#ifndef _spectrum_display_hpp_cm120516_
#define _spectrum_display_hpp_cm120516_

#include "aligned_vector.hpp"
#include "carrier_monitoring/polynomial_interval_fit.hpp"
#include "processor/service.hpp"
#include "Spectrum.hpp"

#include <FL/Fl.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Input.H>
// #include <FL/Fl_draw.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Menu_Bar.H>
//#include <FL/Fl_Menu_Window.H>
#include <FL/Fl_Simple_Counter.H>
#include <FL/Fl_Text_Display.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Value_Input.H>
#include <FL/Fl_Value_Output.H>

#include <vector>
#include <cmath>

#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

class popup_window : public Fl_Double_Window {
public:
  popup_window()
    : Fl_Double_Window(10,10)
    , output_(0, 0, w(), h()) {
    output_.box(FL_UP_BOX);
    end();
    hide();
    border(0);
    output_.align(FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    output_.label("No text defined");
    SizeToText();
  }
  virtual ~popup_window() {}

  // Change text in box
  void text(const char*s) {
    output_.label(s);
    SizeToText();
  }
  // Pop up window at current mouse position
  void popup() {
    position(Fl::event_x_root(), Fl::event_y_root());
    show();
  }

protected:
  // Size window to just fit output's label text
  void SizeToText() {
    int W=0, H=0;
    fl_font(output_.labelfont(), output_.labelsize());
    fl_measure(output_.label(), W, H, 0);
    resize(x(), y(), W+10, H+10);
    output_.resize(0, 0, W+10, H+10);
  }

private:
  Fl_Box output_;
};

class spectrum_display : public Fl_Double_Window {
public:
  enum {
    sMin,
    sMax,
    fMin,
    fMax,
    fCur
  } ;
  typedef std::vector<std::pair<double, double> > zoom_history_type;

  spectrum_display(int x, int y, int w, int h)
    : Fl_Double_Window(x, y, w, h)
    , b0_  (  0, h-20,  50, 20, "Min:")
    , sMin_( 50, h-20,  50, 20, "sMin")
    , b1_  (100, h-20,  50, 20, "Max:")
    , sMax_(150, h-20,  50, 20, "sMax")
    , fMin_(250, h-20,  50, 20, "fMin:")
    , fMax_(350, h-20,  50, 20, "fMax:")
    , fCur_(500, h-20,  80, 20, "fCur:")
    , fStreamName_(700, h-20, 150, 20, "Stream Name:")
    , fTime_(    w-200, h-20, 160, 20, "Current Time:")
    , popup_()
    , xPush_(-1)
    , xPushPeak_(-1)
    , init_(true)
    , spec_fMin_(0)
    , spec_fMax_(40e6) {

    popupText_[0] = 0;

    sMin_.step(1,1); sMin_.range(-140,0); sMin_.value(-70);
    sMax_.step(1,1); sMax_.range(-140,0); sMax_.value(-40);

    fMin_.value( 0); fMax_.value(40e3);
    fCur_.value(-1); fCur_.precision(1); fCur_.range(0, 40e3);
    fMin_.when(FL_WHEN_ENTER_KEY);
    fMax_.when(FL_WHEN_ENTER_KEY);

    sMin_.callback(&cb, sMin);
    sMax_.callback(&cb, sMax);

    fMin_.callback(&cb, fMin);
    fMax_.callback(&cb, fMax);

    fMin_.range(0, 40e3);
    fMax_.range(0, 40e3);

    end();
    specIndex_ = specM();
    spec_.resize(specN() * specM(), -120);
    spec_fitted_.resize(specN() * specM(), -120);
    specImg_.resize(2 * 3 * specN() * specM(), 0);
    exists_ = true;
  }
  virtual ~spectrum_display() {
    exists_ = false;
  }

  static void cb(Fl_Widget *w, long u) {
    spectrum_display* s(dynamic_cast<spectrum_display*>(w->parent()));
    const double v(((Fl_Valuator*)(w))->value());
    switch (u) {
    case sMin: s->sMax_.minimum(v+2);
      break; 
    case sMax: s->sMin_.maximum(v-2);
      break;
     case fMin:
       if (v < 0   ) s->fMin_.value(1.);
       if (v > 40e3) s->fMin_.value(40e3-1.);
       if (s->fMin_.value() >= s->fMax_.value())
         s->fMin_.value(s->fMax_.value()-1);
       break;
    case fMax:
       if (v < 0   ) s->fMax_.value(1.);
       if (v > 40e3) s->fMax_.value(40e3-1);
       if (s->fMin_.value() >= s->fMax_.value())
         s->fMax_.value(s->fMin_.value()+1);
      break;
    }
    s->damage(FL_DAMAGE_ALL);
  }
  
  template<typename T>
  void insert_spec(const frequency_vector<T>& spec,
                   const frequency_vector<T>& spec_filtered,
                   processor::service_iq::sptr sp,
                   bool update_window=true) {

    spec_fMin_ = spec.fmin();
    spec_fMax_ = spec.fmax();
    set_fMin(std::min(std::max(get_fMin(), spec.fmin()), spec.fmax()));
    set_fMax(std::max(std::min(get_fMax(), spec.fmax()), get_fMin()+20));

    if (sp) {
      fStreamName_.value(sp->stream_name().c_str());
      
      std::stringstream oss;
      oss.imbue(std::locale(oss.getloc(), 
                            new boost::posix_time::time_facet("%Y-%m-%d %H:%M:%S")));
      oss << sp->approx_ptime();
      fTime_.value(oss.str().c_str());

      // peak finding
      if (xPushPeak_ != -1) {
        const double fPush = xInputFromSpec(xPushPeak_);
        const int    iPush = spec_filtered.freq2index(fPush);
        const double sPush = spec_filtered[iPush].second;
        const int    sfSize = spec_filtered.size();
        if (iPush > 0 && iPush+1 < sfSize) {
          bool dir[3] =  {
            sPush > spec_filtered[iPush+1].second,
            sPush > spec_filtered[iPush-1].second,
            false
          };
#if 0
          std::cout << "=== ["
                    << spec_filtered[iPush-1].second << " "
                    << spec_filtered[iPush  ].second << " "
                    << spec_filtered[iPush+1].second << "] dir="
                    << dir[0] << dir[1] << std::endl;
#endif
          int ip[3] = { -1, -1, -1 };
          for (int i=iPush; i+1<sfSize; ++i) {
            ip[0] = i;
            if (dir[0] != (spec_filtered[i].second > spec_filtered[i+1].second))
              break;            
          }
          // ++ip[0];
          for (int i=iPush; i>1; --i) {
            ip[1] = i;
            if (dir[1] != (spec_filtered[i].second > spec_filtered[i-1].second))
              break;
          }
          // --ip[1];
          // sort
          if (spec_filtered[ip[0]].second < spec_filtered[ip[1]].second) {
            std::swap(ip[0],  ip[1]);
            std::swap(dir[0], dir[1]);
          }
          if (ip[0] < ip[1]) {
            for (int i=ip[0]; i>1; --i) {
              ip[2] = i;
              if (spec_filtered[i].second < spec_filtered[i-1].second)
                break;
            }
            // --ip[2];
          } else {
            for (int i=ip[0]; i<sfSize-1; ++i) {
              ip[2] = i;
              if (spec_filtered[i].second < spec_filtered[i+1].second)
                break;
            }
            // ++ip[2];
          }

          std::sort(ip, ip+3);
          const int dist = std::min(3, std::min(ip[1]-ip[0], ip[2]-ip[1]));
          const int range[2] = {
            ip[1]-dist,
            ip[1]+dist
          };
          double sum_w=0, sum_wx=0;
          for (int i=range[0]; i<=range[1]; ++i) {
            const double w = std::pow(10.0, 0.1*spec_filtered[i].second);
            sum_w  += w ;
            sum_wx += w*spec_filtered[i].first;
            // std::cout << "--- " << i << " " << spec_filtered[i].first << " " << spec_filtered[i].second << " " << w << std::endl;
          }
#if 0
          std::cout << "fPush= " << fPush << " " << spec_filtered[iPush].first
                    << " dir= " << dir[0] << dir[1] << dir[2]
                    << " f= "
                    << spec_filtered[ip[0]].first << " "
                    << spec_filtered[ip[1]].first << " "
                    << spec_filtered[ip[2]].first << " s="
                    << spec_filtered[ip[0]].second << " "
                    << spec_filtered[ip[1]].second << " "
                    << spec_filtered[ip[2]].second << " ===> " << sum_wx/sum_w
                    << std::endl;
#endif
          sprintf(popupText_, "%.1f (%d)", sum_wx/sum_w, dist);
          if (update_window)
            popup_.text(popupText_);
        } else {
          if (update_window) {
            popupText_[0] = 0;
            popup_.hide();
          }
        }
      }
    }
    std::vector<T> s(specN(),  0);
    std::vector<T> sf(specN(), 0);
    std::vector<int> counters(specN(), 0);
    for (size_t i(0), n(spec.size()); i<n; ++i) {
      const int j(xSpecFromInput(spec[i].first)-xSpecBeg());
      s[j]  = (0 == counters[j]) ? spec[i].second          : std::max(s[j],  spec[i].second);
      sf[j] = (0 == counters[j]) ? spec_filtered[i].second : std::max(sf[j], spec_filtered[i].second);
      ++counters[j];
    }
    const double dfn(delta_f() / double(specN()));
    for (int j(0), n(specN()); j<n; ++j) {
      if (counters[j] != 0) 
        continue;
      const int i(spec.freq2index(get_fMin() + dfn*j));
      s[j]  = spec         [i].second;
      sf[j] = spec_filtered[i].second;
    }
    insert_spec(s, sf, update_window);
  }

  template<typename T>
  void insert_spec(const std::vector<T>& spec,
                   const std::vector<T>& spec_filtered,
                   bool update_window=true) {
    specIndex_--;
    if (specIndex_ < 0) specIndex_ = specM()-1;

    // compute the fitted spectrum
    const size_t n(spec.size());
    std::vector<size_t> b(n, 1);

    const double threshold_db(2);
    const unsigned poly_degree(2);
    
    const size_t m(15); // number of fit intervals
    std::vector<double> indices(m+1, 0);
    for (size_t i(0); i<m; ++i)
      indices[i] = (i*n)/m;

    indices[m] = n-1;
    indices[0] = std::min(indices[1],   indices[0]+size_t(0.005*n));
    indices[m] = std::max(indices[m-1], indices[m]-size_t(0.005*n));

    polynomial_interval_fit p(poly_degree, indices);
    std::vector<double> ts(n, 0);
    for (size_t i(0); i<n; ++i)
      ts[i] = i;

    for (size_t l(0); l<100; ++l) {
      if (!p.fit(ts, spec_filtered, b)) {
	std::cerr << "fit failed" << std::endl;
      }
      size_t nchanged(0);
      for (size_t i(0); i<n; ++i) {
	const std::pair<double,double> vf(p.eval(i));
	const bool c(spec_filtered[i]-vf.first > threshold_db);
	nchanged += (c==b[i]);
	b[i] = !c;
      }
      if (0 == nchanged)
	break;
    }

    for (size_t i(0); i<n; ++i) {
      spec_[specIndex_*specN()+i] = spec_filtered[i];

      const std::pair<double,double> vf(p.eval(i));
      spec_fitted_[specIndex_*specN()+i] = vf.first;

      // normalized, clamped signal
      const double x(clamp((spec[i]-sMin_.value())/(sMax_.value()-sMin_.value())));
//       const double x(clamp((spec[i]-vf.first+2.5)/(sMax_.value()-sMin_.value())));

      const size_t j1(3*(specIndex_*specN() + i));
      const size_t j2(j1 + 3*specM()*specN());
      const size_t colorMapIndex(colorScale((unsigned char)(x*22)));
      specImg_[j1]   = specImg_[j2]   = svgaPalette(3*colorMapIndex+2)<<2;
      specImg_[j1+1] = specImg_[j2+1] = svgaPalette(3*colorMapIndex+1)<<2;
      specImg_[j1+2] = specImg_[j2+2] = svgaPalette(3*colorMapIndex+0)<<2;

    }
    if (update_window)
      this->damage(FL_DAMAGE_ALL);
    init_ = false;
  }

  // clamp x \in [0,1]
  static double clamp(double x) {
    return (x<0) ? 0 : (x>1) ? 1 : x;
  }

  void draw() {
    Fl_Widget *const*a = array();
    if (damage() == FL_DAMAGE_CHILD) { // only redraw some children
      for (int i = children(); i--; a++)
	update_child(**a);
    } else { // total redraw
      fl_draw_box(FL_FLAT_BOX, 0, 0, w(), h(), FL_GRAY);
      fStreamName_.value();
      draw_spec();
      draw_waterfall();
      draw_frames();
      draw_ticks(tick_stepsize());

      // now draw all the children atop the background:
      for (int i = children(); i--; a++) {
	draw_child(**a);
	draw_outside_label(**a); // you may not need to do this
      }
    }
  }

  int tick_stepsize() const {
    static int f[3] = { 1, 2, 5 };
    const double df(delta_f());
    int x(1);
    for (size_t i(0); i<10; ++i) {
      for (size_t j(0), n(sizeof(f)/sizeof(int)); j<n; ++j)
        if (df/(f[j]*x) < 20.) return f[j]*x;
      x *= 10;
    }
    return std::min(x, 20);
  }

  void draw_spec() const {
    if (init_) return;
    fl_color(FL_BLUE);
    fl_line_style(FL_SOLID | FL_CAP_ROUND | FL_JOIN_ROUND, 2);
    fl_begin_line();
    for (int i(0), n(specN()); i<n; ++i) {
      fl_vertex(xSpecBeg()+i, ySpecFromInput(spec_fitted_[specIndex_*specN() + i]));
    }
    fl_end_line();

    fl_color(FL_RED);
    fl_line_style(FL_SOLID | FL_CAP_ROUND | FL_JOIN_ROUND, 1);
    fl_begin_line();
    for (int i(0), n(specN()); i<n; ++i) {
      fl_vertex(xSpecBeg()+i, ySpecFromInput(spec_[specIndex_*specN() + i]));
    }
    fl_end_line();

    fl_line_style(FL_SOLID, 0);
  }

  void draw_waterfall() const {
    if (init_) return;
    fl_draw_image(&specImg_[0] + 3*specIndex_*specN(),
                  xWaterfallBeg(), yWaterfallBeg(),
                  specN(),         specM());
  }

  void draw_frames() {
    fl_color(FL_BLACK);
    fl_rect(xSpecBeg(),
            ySpecBeg(),
            xSpecEnd()-xSpecBeg(),
            ySpecEnd()-ySpecBeg());
    fl_rect(xWaterfallBeg(),
            yWaterfallBeg(),
            xWaterfallEnd()-xWaterfallBeg(),
            yWaterfallEnd()-yWaterfallBeg());
  }

  // every step kHz
  void draw_ticks(int step) {
    fl_color(FL_BLUE);
    fl_line_style(FL_DASH);
    char label[1024];
    int last_x(-100000);
    for (double fi=step*(int(get_fMin())/step); fi <= get_fMax(); fi += step) {
      if (fi < get_fMin()) continue;
      const int xl = xSpecFromInput(fi);
      if ((step % 1000) == 0)
        sprintf(label, "%.0f", fi/1000);
      else if ((step % 100)==0)
        sprintf(label, "%.1f", fi/1000);
      else if ((step % 10)==0)
        sprintf(label, "%.2f", fi/1000);
      else 
        sprintf(label, "%.3f", fi/1000);
      const double labelWidth = fl_width(label);
      // int _dx(0), _dy(0), _w0(0), _h0(0);
      // fl_text_extents(label, _dx, _dy, _w0, _h0);
      // const double labelHeight(_h0);
      if (xl-labelWidth/2 <= xSpecBeg()) continue;
      if (xl+labelWidth/2 >= xSpecEnd()) continue;
      if (xl-labelWidth/2 < last_x) continue;
      fl_line(xl, ySpecBeg(),      xl, ySpecEnd());
      fl_line(xl, yWaterfallBeg(), xl, yWaterfallEnd());
      fl_draw(label, xl-int(labelWidth/2+0.5), yWaterfallBeg()-fl_descent());
      last_x = xl+int(labelWidth/2+0.5);
    }

    int step_db = 10;
    if (sMax_.value()-sMin_.value() < 30)
      step_db = 5;
    if (sMax_.value()-sMin_.value() < 15)
      step_db = 2;
    if (sMax_.value()-sMin_.value() < 10)
      step_db = 1;

    int last_y(ySpecEnd());
    for (int s(int(sMin_.value())); s<sMax_.value(); s+= step_db) {
      const int yl(ySpecFromInput(s));
      sprintf(label, "%d", s);
      int _dx(0), _dy(0), _w0(0), _h0(0);
      fl_text_extents(label, _dx, _dy, _w0, _h0);
      const double labelHeight(_h0);
      const double labelWidth(_w0);
      if ((yl+labelHeight/2 >= ySpecEnd()) ||
          (yl-labelHeight/2 <= ySpecBeg()) || 
          (last_y - (yl+labelHeight/2) < 2))
        continue;
      fl_line(xSpecBeg(), yl, xSpecEnd(), yl);
      fl_draw(label, xSpecBeg()-int(labelWidth+0.5)-4, yl+int(labelHeight/2+0.5));
      last_y = yl-int(labelHeight/2+0.5);
    }
  }

  int handle(int event) {
    if (!exists_) return Fl_Double_Window::handle(event);
//     std::cout << "handle: " << event << " " << this << " " << std::endl;
    switch (event) {
    case FL_MOVE: {
      fCur_.value(0);
      if (xSpecBeg() < Fl::event_x() && Fl::event_x() < xSpecEnd()) {
 	if ((ySpecBeg()      < Fl::event_y() && Fl::event_y() < ySpecEnd()) ||
 	    (yWaterfallBeg() < Fl::event_y() && Fl::event_y() < yWaterfallEnd()))	  
 	  fCur_.value(xInputFromSpec(Fl::event_x()));
      }
      return Fl_Double_Window::handle(event);
    }
    case FL_PUSH: {
      if(callback() && (when() & FL_WHEN_CHANGED))
        do_callback();
      if (isInSpec     (Fl::event_x(), Fl::event_y()) ||
          isInWaterfall(Fl::event_x(), Fl::event_y())) {
        if (Fl::event_button() == FL_LEFT_MOUSE) {
          xPush_ = Fl::event_x();
          // std::cout << "handle: FL_PUSH " << event
          //           << " x,y= " << Fl::event_x() << " " << Fl::event_y() << std::endl;
        }
        if (Fl::event_button() == FL_RIGHT_MOUSE) {
          xPushPeak_ = Fl::event_x();
          popup_.popup();
        }
        return 1;
      }
      return Fl_Double_Window::handle(event);
    }
    case FL_RELEASE: {
      if(callback() && (when() & FL_WHEN_CHANGED))
        do_callback();
      if (isInSpec     (Fl::event_x(), Fl::event_y()) ||
          isInWaterfall(Fl::event_x(), Fl::event_y())) {
        if (Fl::event_button() == FL_LEFT_MOUSE) {
          // std::cout << "handle: FL_RELEASE " << event
          //           << " x,y= " << Fl::event_x() << " " << Fl::event_y() << std::endl;
          if (Fl::event_x() != xPush_) {
            double f[2] = {
              xInputFromSpec(xPush_),
              xInputFromSpec(Fl::event_x())
            };
            std::sort(f, f+2);
            set_fMin(f[0]);
            set_fMax(f[1]);
            // std::cout << "x1,x2= " << xPush_ << " " << Fl::event_x() << std::endl;
            // std::cout << "f1,f2= " << f[0] << " " << f[1] << std::endl;
            zoom_hist_.push_back(std::make_pair(f[0], f[1]));
          }
        }
        if (Fl::event_button() == FL_RIGHT_MOUSE) {
          //          popup_.hide();
        }
        return 1;
      }
      return Fl_Double_Window::handle(event);
    }
    case FL_KEYDOWN: {
      if(callback() && (when() & FL_WHEN_CHANGED))
        do_callback();
      if (isInSpec     (Fl::event_x(), Fl::event_y()) ||
          isInWaterfall(Fl::event_x(), Fl::event_y())) {
        const char* txt(Fl::event_text());
        // std::cout << "key pressed: " << txt << " '" << txt[0]<< "' len=" << Fl::event_length() << std::endl;
        if (Fl::event_length()>0 && txt[0] == 'u') {
          // unzoom
          if (!zoom_hist_.empty())
            zoom_hist_.pop_back();
          if (!zoom_hist_.empty()) {
            const std::pair<double,double> fr(zoom_hist_.back());
            set_fMin(fr.first);
            set_fMax(fr.second);
          } else {
            set_fMin(0);
            set_fMax(40e6);
          }
        }
      }
      return Fl_Double_Window::handle(event);
    }
    default:
      return Fl_Double_Window::handle(event);
    }
  }

  // kHz -> Hz
  double get_fMin() const { return 1e3*fMin_.value(); }
  double get_fMax() const { return 1e3*fMax_.value(); }
  double delta_f() const { return get_fMax() - get_fMin(); }
  // Hz -> kHz
  double set_fMin(double fMin) { return fMin_.value(1e-3*fMin); }
  double set_fMax(double fMax) { return fMax_.value(1e-3*fMax); }

  double set_sMin(double sMin) { return sMin_.value(sMin); }
  double set_sMax(double sMax) { return sMax_.value(sMax); }

protected:
  int ySpecBeg() const { return  0; }
  int ySpecEnd() const { return 100; }
  int yWaterfallBeg() const { return ySpecEnd()+16; }
  int yWaterfallEnd() const { return h()-20; }

  int xSpecBeg() const { return  30; }
  int xSpecEnd() const { return w(); }
  int xWaterfallBeg() const { return xSpecBeg(); }
  int xWaterfallEnd() const { return xSpecEnd(); }

  int specN() const { return xSpecEnd()      - xSpecBeg(); }
  int specM() const { return yWaterfallEnd() - yWaterfallBeg(); }

  int xSpecFromInput(double xi) const {
    double x = (xi-get_fMin()) / delta_f();
    x = (x<0) ? 0. : (x>1) ? 1. : x;
    return int(0.5+ xSpecBeg() + x * (xSpecEnd()-xSpecBeg()-1));
  }
  int ySpecFromInput(double yi) const {
    double x = (yi-sMin_.value()) / (sMax_.value()-sMin_.value());
    x = (x<0) ? 0. : (x>1) ? 1. : x;
    return int(0.5+ ySpecEnd()-1 - x * (ySpecEnd()-ySpecBeg()));
  }
  double xInputFromSpec(int i) const {
    double x = double(i-xSpecBeg())/double(xSpecEnd()-xSpecBeg());
    return (x<0) ? -1 : (x>1) ? -1 : get_fMin() + x * delta_f();
  }

  bool isInSpec(int x, int y) const {
    return (x > xSpecBeg() && x < xSpecEnd() &&
            y > ySpecBeg() && y < ySpecEnd());
  }
  bool isInWaterfall(int x, int y) const {
    return (x > xWaterfallBeg() && x < xWaterfallEnd() &&
            y > yWaterfallBeg() && y < yWaterfallEnd());
  }

  // adapted from palette.c \in linrad
  static unsigned char colorScale(size_t index) {
    static unsigned char color_scale[]={ 
      0, 36,37,38,39,40,41,42,43,44,
      45,46,47,48,49,50,51,52,53,54,
      55,15,15};
    static size_t len(sizeof(color_scale)/sizeof(unsigned char));
    return color_scale[(index>len) ? len-1 : index];
  }
  // adapted from palette.c \in linrad
  static unsigned char svgaPalette(size_t index) {
    static unsigned char svga_palette[] = {
      //    0       |      1       |     2        |     3
      0x00,0x00,0x00,0x2a,0x00,0x00,0x00,0x2a,0x00,0x2a,0x2a,0x00,
      //    4       |      5       |     6        |     7
      0x00,0x00,0x2a,0x2a,0x00,0x2a,0x00,0x15,0x2a,0x2a,0x2a,0x2a,
      //    8       |      9       |     10       |     11
      0x15,0x15,0x15,0x3f,0x15,0x15,0x15,0x3f,0x15,0x3f,0x3f,0x15,
      //    12      |      13      |     14       |     15
      0x15,0x15,0x3f,0x3f,0x15,0x3f,0x15,0x3f,0x3f,0x3f,0x3f,0x3f,
      //    16      |      17      |     18       |     19
      0x00,0x17,0x00,0x05,0x05,0x05,0x08,0x08,0x08,0x0b,0x0b,0x0b,
      //    20      |      21      |     22       |     23
      0x0e,0x0e,0x0e,0x11,0x11,0x11,0x14,0x14,0x14,0x18,0x18,0x18,
      //    24      |      25      |     26       |     27
      0x1c,0x1c,0x1c,0x20,0x20,0x20,0x24,0x24,0x24,0x28,0x28,0x28,
      //    28      |      29      |     30       |     31
      0x2d,0x2d,0x2d,0x32,0x32,0x32,0x38,0x38,0x38,0x08,0x18,0x1a,
      //    32      |      33      |     34       |     35
      0x20,0x3f,0x20,0x12,0x00,0x00,0x17,0x00,0x00,0x00,0x00,0x17,
      //    36      |      37      |     38       |     39
      0x08,0x00,0x00,0x10,0x00,0x00,0x18,0x00,0x00,0x1a,0x0a,0x00,
      //    40      |      41      |     42       |     43
      0x1c,0x0f,0x00,0x1d,0x16,0x00,0x17,0x1c,0x00,0x14,0x21,0x00,
      //    44      |      45      |     46       |     47
      0x05,0x23,0x00,0x00,0x28,0x0a,0x00,0x28,0x1e,0x00,0x23,0x25,
      //    48      |      49      |     50       |     51
      0x00,0x21,0x30,0x00,0x1c,0x32,0x00,0x16,0x34,0x00,0x0f,0x3a,
      //    52      |      53      |     54       |     55
      0x0a,0x0a,0x3f,0x14,0x14,0x3f,0x1f,0x1f,0x3f,0x2b,0x2b,0x3f,
      //    56      |      57      |     58       |     59
      0x0e,0x29,0x22,0x12,0x12,   0,0x30,0x30,0x3f,         0,0,0};
    static size_t len(sizeof(svga_palette)/sizeof(unsigned char));
    return svga_palette[(index>len) ? len-1 : index];
  }

private:
  Fl_Box              b0_;
  Fl_Simple_Counter   sMin_;
  Fl_Box              b1_;
  Fl_Simple_Counter   sMax_;
  Fl_Value_Input      fMin_;
  Fl_Value_Input      fMax_;
  Fl_Value_Output     fCur_;
  Fl_Output           fStreamName_;
  Fl_Output           fTime_;
  popup_window        popup_;
  char                popupText_[1024];
  int                 specIndex_;
  int                 xPush_;
  int                 xPushPeak_;
  std::vector<double> spec_;
  std::vector<double> spec_fitted_;
  std::vector<uchar>  specImg_;
  bool                init_;
  bool exists_;
  double              spec_fMin_;
  double              spec_fMax_;
  zoom_history_type   zoom_hist_;
} ;

#endif // _spectrum_display_hpp_cm120516_
