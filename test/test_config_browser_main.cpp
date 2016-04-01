// -*- C++ -*-
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

#include <complex>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/property_tree/xml_parser.hpp>


#include <FL/Fl.H>
#include <FL/Fl_Tree.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Text_Display.H>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <iostream>
#include <vector>
#include <cmath>

#include "logging.hpp"

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

/// FLTK config browser
class MyWindow : public Fl_Double_Window {
public:
  typedef boost::property_tree::ptree ptree;

  MyWindow(int w, int h, const ptree& config)
    : Fl_Double_Window(w, h, "config browser")
    , menu_bar_(0,0,w,20)
    , tree_(0,20,w,h-20, "config browser")
  {
    menu_bar_.add("File/Quit", FL_CTRL+'q', Quit_CB);

    browse_ptree(config, "");
    end();
    this->callback(Quit_CB);
  }
  virtual ~MyWindow() {}

protected:

  void browse_ptree(const ptree& pt, std::string path) {
    BOOST_FOREACH(const ptree::value_type &v, pt) {
      std::cout << "config: " << v.first << std::endl;
      if (!v.second.empty()) {
	browse_ptree(v.second, path+"/"+v.first);
      } else {
	std::cout << "leave: " << path << std::endl;
	tree_.add(path.c_str()+1);
      }
    }
  }

  static void Quit_CB(Fl_Widget *, void *) {
    Fl::lock();
    static char msg[1024];
    sprintf(msg, "quit");
    Fl::awake(msg);
    Fl::unlock();
  }
private:
  Fl_Menu_Bar menu_bar_;
  Fl_Tree tree_;
} ;

int main(int argc, char* argv[])
{
  // set up command-line options
  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,?",    "produce help message")
    ("version,v", "display version")
    ("config,c",  po::value<std::string>(), "path to XML configuration file");

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
  LOGGER_INIT("./Log", "test_config_browser");

  try {
    boost::property_tree::ptree config;
    std::cout << "using config file: " << vm["config"].as<std::string>() << std::endl;
    read_xml(vm["config"].as<std::string>(), config);

    // FLTK initialization
    Fl::visual(FL_RGB);
    MyWindow w(400,600, config);
    w.show();
    while (Fl::wait() > 0) {
      const char* msg(static_cast<const char *>(Fl::thread_message()));
      if (msg) {
	if (std::string(msg) == "quit") break;
      }
    }    
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
