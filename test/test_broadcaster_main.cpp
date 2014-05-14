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
#include <iostream>
#include <boost/property_tree/xml_parser.hpp>

#include "network/broadcaster.hpp"
#include "network/protocol.hpp"
#include "run.hpp"

class test_bc {
public:
  typedef boost::shared_ptr<test_bc> sptr;
  typedef network::broadcaster::data_connection::ptime ptime;
  typedef std::string data_type;
  
  test_bc(const boost::property_tree::ptree& config)
    : bc_(network::broadcaster::broadcaster::make(config)) {
  }

  static sptr make(const boost::property_tree::ptree& config) {
    return sptr(new test_bc(config));
  }

  void start() { bc_->start(); }
  void stop() { bc_->stop(); }

  void callback() {
    LOG_INFO("callback");
    last_callback_time_ = boost::posix_time::microsec_clock::universal_time();
    std::string message1("Hi There");
    std::string message2("Hello World!");
    std::string message3("Howdy?");

    network::protocol::header h1("TEST0001", last_callback_time_, 0, message1.size());
    std::string data1;
    std::copy(h1.begin(), h1.end(), std::back_inserter(data1));
    std::copy(message1.begin(), message1.end(), std::back_inserter(data1));

    network::protocol::header h2("TEST0002", last_callback_time_, 0, message2.size());
    std::string data2;
    std::copy(h2.begin(), h2.end(), std::back_inserter(data2));
    std::copy(message2.begin(), message2.end(), std::back_inserter(data2));

    network::protocol::header h3("TEST0003", last_callback_time_, 0, message3.size());
    std::string data3;
    std::copy(h3.begin(), h3.end(), std::back_inserter(data3));
    std::copy(message3.begin(), message3.end(), std::back_inserter(data3));

    bc_->bc_data(last_callback_time_, "data_00001", "TEST0001", message1);
    bc_->bc_data(last_callback_time_, "data_00002", "TEST0002", message2);
    bc_->bc_data(last_callback_time_, "data_00003", "TEST0003", message3);
  }

protected:
private:
  network::broadcaster::broadcaster::sptr bc_;
  ptime last_callback_time_;
} ;

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_broadcaster");
  try {
    const std::string filename((argc > 1 ) ? argv[1] : "config.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config);

    test_bc::sptr bp = test_bc::make(config);
    bp->start();
    const size_t n(100);
    for (size_t i=0; i<n; ++i) {
      bp->callback();
      usleep(1000*1000);
    }
    bp->stop();

  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
