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
#include <complex>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/enable_shared_from_this.hpp>

#include "logging.hpp"
#include "libusb1.hpp"
#include "portaudio.hpp"
#include "fifi_control.hpp"
#include "logging.hpp"
#include "broadcaster.hpp"
#include "protocol.hpp"

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
    throw std::runtime_error(str(boost::format("Audio Device '%s' not found.") % device_name));
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

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

class fifi_bc : public broadcaster,
                public portaudio::stream_callback::process_base,
                public boost::enable_shared_from_this<fifi_bc> {
public:
  typedef boost::shared_ptr<fifi_bc> sptr;

  fifi_bc(boost::asio::io_service&           io_service,
          const boost::property_tree::ptree& config,
          portaudio::device_info::sptr       audio_device,
          double                             sample_rate,
          size_t                             frames)
    : broadcaster(io_service, config)
    , input_parameters_(portaudio::stream_parameters::make
                        (audio_device->index(),
                         2,
                         paFloat32,
                         audio_device->default_high_input_latency()))
    , output_parameters_()
    , sample_rate_(sample_rate)
    , frames_(frames) {
      if (not portaudio::session::get_global_session()->is_format_supported
          (input_parameters_, output_parameters_, sample_rate_))
        throw std::runtime_error("audio format is not supported");
    }
  
  virtual ~fifi_bc() {
    // in case it is still around
    stream_callback_.reset();
  }

  // portaudio callback
  virtual int process(const void *input_buffer, 
                      void *output_buffer,
                      unsigned long frames_per_buffer,
                      portaudio::callback_info::sptr info) {
    const header header_info("IQIn", // IQ Info
                             boost::posix_time::microsec_clock::universal_time(),
                             sizeof(iq_info));
    const iq_info info_iq(sample_rate_,
                          0, // ddc_center_frequecy_Hz
                          'f', sizeof(float));

    const size_t buf_size(frames_per_buffer * sizeof(std::complex<float>));
    const header header_iq("IQDa", // IQ Data
                           header_info.approx_ptime(),
                           buf_size);

    std::string data;
    std::copy((char*)&header_info,  (char*)&header_info + sizeof(header),  std::back_inserter(data));
    std::copy((char*)&info_iq,      (char*)&info_iq     + sizeof(iq_info), std::back_inserter(data));
    std::copy((char*)&header_iq,    (char*)&header_iq   + sizeof(header),  std::back_inserter(data));
    std::copy((char*)input_buffer,  (char*)input_buffer + buf_size,        std::back_inserter(data));
    bc_data(header_info.approx_ptime(), "iq", data);
    do_pending_io_actions();
    // return (counter_ == num_average_) ? paAbort : paContinue;
    return paContinue;
  }

  virtual void start_data_source() {
    LOG_INFO("start_data_source");
    stream_callback_ = portaudio::stream_callback::make(input_parameters_,
                                                        output_parameters_,
                                                        sample_rate_,
                                                        frames_,
                                                        paNoFlag, 
                                                        shared_from_this());
    stream_callback_->start();
  }

  virtual void stop_data_source() {
    LOG_INFO("stop_data_source");
    get_strand().wrap(boost::bind(&fifi_bc::do_stop_stream, this));
    do_pending_io_actions();
    stream_callback_.reset();
  }
protected:
private:
  void do_stop_stream() {
    LOG_INFO("do_stop_stream");
    stream_callback_->stop();
  }

  portaudio::stream_parameters::sptr input_parameters_;
  portaudio::stream_parameters::sptr output_parameters_;
  double sample_rate_;
  size_t frames_;
  portaudio::stream_callback::sptr stream_callback_;
} ;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
  LOGGER_INIT("./Log", argv[0]); 
  try {
    std::string filename((argc > 1 ) ? argv[1] : "config.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config, boost::property_tree::xml_parser::no_comments);

    const boost::property_tree::ptree&
      config_fifi_sdr(config.get_child("FiFiDAQ.FiFi-SDR"));

    // global portaudio session
    const portaudio::session::sptr pa(portaudio::session::get_global_session());

    usb_device_handle::sptr
      ud(find_usb_device(config_fifi_sdr));
    portaudio::device_info::sptr
      ad(find_portaudio_device(config_fifi_sdr));    

    FiFiSDR::receiver_control::sptr
      rec(FiFiSDR::receiver_control::make(ud));
    
    // initialize the receiver
    rec->init(config_fifi_sdr);

    // install the signal handler
    signal(SIGINT,  sig_handler);
    signal(SIGQUIT, sig_handler);
    signal(SIGTERM, sig_handler);

    boost::asio::io_service io_service;

    const double sample_rate(config_fifi_sdr.get<double>("<xmlattr>.sampleRate_Hz"));

    // this also establishes a global portaudio session needed to run the callback
    fifi_bc::sptr bc(new fifi_bc(io_service,
                                 config.get_child("FiFiDAQ"),
                                 ad,
                                 sample_rate,
                                 size_t(0.1/sample_rate)));

    bc->start();
    while (run())
      pa->sleep(0.05);
    bc->stop();

  } catch (const std::runtime_error& e) {
    LOG_ERROR(e.what());
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
