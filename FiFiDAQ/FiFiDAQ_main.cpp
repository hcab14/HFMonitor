// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>

#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "util.hpp"
#include "libusb1.hpp"
#include "portaudio.hpp"
#include "fifi_control.hpp"

usb_device_handle::sptr find_usb_device(const boost::property_tree::ptree& config_fifi_daq) {
  const unsigned vendor_id (config_fifi_daq.get<unsigned>("<xmlattr>.vendorID",  0));
  const unsigned product_id(config_fifi_daq.get<unsigned>("<xmlattr>.productID", 0));
  const std::vector<usb_device_handle::sptr> dl(usb_device_handle::get_device_list(vendor_id, product_id));
  if (dl.size() != 1) {
    const std::vector<usb_device_handle::sptr> dl(usb_device_handle::get_device_list(0, 0));
    std::cerr << "Available USB devices:\n";
    for (size_t i=0; i<dl.size(); ++i) {
      std::cerr << (boost::format("(%2d) vendor_id=%5d product_id=%5d serial='%s'") 
                    % i
                    % dl[i]->get_vendor_id() 
                    % dl[i]->get_product_id()
                    % dl[i]->get_serial()) << std::endl;
    }
    throw std::runtime_error(str(boost::format("FiFi-SDR (vendor_id=%5d, product_id=%5d) not found.") 
                                 % vendor_id % product_id));
  } else
    return dl[0];
}

portaudio::device_info::sptr find_portaudio_device(const boost::property_tree::ptree& config_fifi_daq,
                                                   portaudio::init::sptr pa) {
  const std::string device_name(config_fifi_daq.get<std::string>("<xmlattr>.audioDeviceName", "UDA1361 Eingang"));
  const portaudio::init::device_list dl(pa->get_device_list(device_name));
  if (dl.size() != 1) {
    const portaudio::init::device_list dl(pa->get_device_list());
    std::cerr << "Audio Device List:\n";
    for (size_t i(0); i<dl.size(); ++i)
      std::cerr << (boost::format("(%2d) '%s'") % i % dl[i]->name()) << std::endl;
    throw std::runtime_error(str(boost::format("Audio Device %s not found.") % device_name));
  } else
    return dl[0];
}

int main(int argc, char* argv[])
{
  try {
    std::string filename((argc > 1 ) ? argv[1] : "config.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config, boost::property_tree::xml_parser::no_comments);
    const boost::property_tree::ptree& config_fifi_daq(config.get_child("FiFiDAQ.FiFi-SDR"));

    portaudio::init::sptr pa(portaudio::init::make());

    usb_device_handle::sptr      ud(find_usb_device(config_fifi_daq));
    portaudio::device_info::sptr ad(find_portaudio_device(config_fifi_daq, pa));    

    FiFiSDR::receiver_control::sptr rec(FiFiSDR::receiver_control::make(ud));
    std::cout << "frequency = " << rec->get_frequency() << std::endl;
    rec->set_frequency(10.0);
    std::cout << "frequency = " << rec->get_frequency() << std::endl;

    std::cout << "xtal frequency = " << rec->get_xtal_frequency() << std::endl;
    rec->set_xtal_frequency(rec->get_xtal_frequency());
    std::cout << "xtal frequency = " << rec->get_xtal_frequency() << std::endl;

    FiFiSDR::receiver_control::presel_entry pe = rec->get_presel_entry(0);
    std::cout << pe.freq1() << " "<< pe.freq2() << " " << rec->num_abpf() << std::endl;

  } catch (const std::runtime_error& e) {
    std::cerr << e.what() << std::endl;
  }
}
