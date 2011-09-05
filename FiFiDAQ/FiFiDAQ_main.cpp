// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2011 Christoph Mayer
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
#include <iostream>
#include <stdlib.h>
#include <signal.h>
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
#include "logging.hpp"
#include "mat_spectrum_saver.hpp"
#include "pa_fft.hpp"

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

portaudio::device_info::sptr find_portaudio_device(const boost::property_tree::ptree& config_fifi_daq) {
  const std::string device_name(config_fifi_daq.get<std::string>("<xmlattr>.audioDeviceName", "UDA1361 Eingang"));
  const portaudio::session::sptr pa(portaudio::session::get_global_session());
  const portaudio::session::device_list dl(pa->get_device_list(device_name));
  if (dl.size() != 1) {
    std::cerr << "Audio Device List:\n";
    const portaudio::session::device_list dl(pa->get_device_list());
    for (size_t i(0); i<dl.size(); ++i)
      std::cerr << (boost::format("(%2d) '%s'") % i % dl[i]->name()) << std::endl;
    throw std::runtime_error(str(boost::format("Audio Device %s not found.") % device_name));
  } else
    return dl[0];
}

// keeps a global state variable which determines if the program should run or not 
bool& run() {
  static bool _run = true;
  return _run;
}

// signal handler called by SIG{INT,QUIT,TERM}
void sig_handler(int) {
  LOG_INFO("User exit\n");
  run() = false;
  signal(SIGINT,  SIG_DFL);
  signal(SIGQUIT, SIG_DFL);
  signal(SIGTERM, SIG_DFL);
}

int main(int argc, char* argv[]) {
  LOGGER_INIT("./Log", argv[0]); 
  try {
    std::string filename((argc > 1 ) ? argv[1] : "config.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config, boost::property_tree::xml_parser::no_comments);
    const boost::property_tree::ptree& config_fifi_daq(config.get_child("FiFiDAQ.FiFi-SDR"));
    const boost::property_tree::ptree& config_saver(config.get_child("FiFiDAQ.MatSaver"));
    const boost::property_tree::ptree& config_actions(config.get_child("FiFiDAQ.Actions"));

    usb_device_handle::sptr
      ud(find_usb_device(config_fifi_daq));
    portaudio::device_info::sptr
      ad(find_portaudio_device(config_fifi_daq));    

    FiFiSDR::receiver_control::sptr
      rec(FiFiSDR::receiver_control::make(ud));

    portaudio::stream_parameters::sptr 
      input_parameters(portaudio::stream_parameters::make(ad->index(),
                                                          2,
                                                          paFloat32,
                                                          ad->default_high_input_latency()));
    portaudio::stream_parameters::sptr output_parameters;
    const double sample_rate(config_fifi_daq.get<double>("<xmlattr>.sampleRate_Hz"));

    const portaudio::session::sptr
      pa(portaudio::session::get_global_session());

    if (not pa->is_format_supported(input_parameters, output_parameters, sample_rate))
      throw std::runtime_error("audio format is not supported");

    // install the signal handler
    signal(SIGINT,  sig_handler);
    signal(SIGQUIT, sig_handler);
    signal(SIGTERM, sig_handler);

    std::map<std::string, pa_fft::sptr> procs;
    while (run()) {
      BOOST_FOREACH(const boost::property_tree::ptree::value_type& action, config_actions) {
        const std::string action_name(action.second.get<std::string>("<xmlattr>.label"));
        LOG_INFO(str(boost::format("action='%s' name='%s'") % action.first % action_name));
        // for now there is only one type of action
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
          while (sb->is_active() && run())
            pa->sleep(0.05);
          sb->stop();
        } else
          throw std::runtime_error("unknown action " + action.first);
      }
    }
  } catch (const std::runtime_error& e) {
    LOG_ERROR(e.what());
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
