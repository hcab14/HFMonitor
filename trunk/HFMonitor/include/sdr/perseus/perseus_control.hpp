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
#ifndef _perseus_control_hpp_cm120317_
#define _perseus_control_hpp_cm120317_

#include <boost/property_tree/ptree.hpp>

#include "libusb1.0/libusb1.hpp"
#include "logging.hpp"

namespace Perseus {

  // Microtelecom product id data structure
  class product_id {
  public:
    product_id()
      : _sn(0)
      , _prodcode(0)
      , _hwrel(0)
      , _hwver(0) {
      for (size_t i=0; i<6; ++i)
        _signature[i] = 0;
    }
    boost::uint16_t sn() const { return _sn; }
    boost::uint16_t prodcode() const { return _prodcode; }
    boost::uint8_t  hwrel() const { return _hwrel; }
    boost::uint8_t  hwver() const { return _hwver; }

    std::string signature() const { 
      return str(boost::format("%02hX%02X-%02X%02X-%02X%02X")
                 % int(_signature[5]) % int(_signature[4])
                 % int(_signature[3]) % int(_signature[2])
                 % int(_signature[1]) % int(_signature[0]));
      }
    std::string to_str() const {
      return str(boost::format("%05d-%s 0x%04X v%d.%d") % sn() % signature()
                 % prodcode() % int(hwrel()) % int(hwver()));
    }
  private:
    boost::uint16_t  _sn;             // Receiver Serial Number
    boost::uint16_t  _prodcode;       // Microtelecom Product Code
    boost::uint8_t   _hwrel;          // Product release
    boost::uint8_t   _hwver;          // Product version
    boost::uint8_t   _signature[6];   // Microtelecom Original Product Signature
  } __attribute__((__packed__));
    

  class callback : private boost::noncopyable {
  public:
    typedef boost::shared_ptr<callback> sptr;
    virtual ~callback() {}
    
    virtual void operator()(unsigned char* data, size_t length) {}
  } ;

  class receiver_control : private boost::noncopyable {
  public:
    typedef boost::shared_ptr<receiver_control> sptr;
    virtual ~receiver_control() {}

    // get number of available perseus
    static size_t get_num_perseus();

    // get product id for perseus
    static product_id get_product_id_at(size_t, std::string firmware="perseus.hex");

    // get perseus at a certain index
    static sptr make(size_t);

    virtual void init(const boost::property_tree::ptree& config) = 0;

    virtual void   set_sample_rate(int sample_rate) = 0;
    virtual int    get_sample_rate() const = 0;

    virtual void   enable_dither(bool) = 0;
    virtual void   enable_preamp(bool) = 0;
    virtual void   start_async_input(callback::sptr) = 0;
    virtual void   stop_async_input() = 0;
    virtual void   use_preselector(bool) = 0;
    virtual double get_center_frequency_hz() const = 0;
    virtual double set_center_freq_hz(double) = 0;
    virtual void   set_attenuator(boost::uint8_t) = 0;

  protected:
  private:
  } ;
} // namespace Perseus
#endif //  _perseus_control_hpp_cm120317_
