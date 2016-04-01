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
#ifndef _CLIENT_MULTI_BASE_HPP_cm140509_
#define _CLIENT_MULTI_BASE_HPP_cm140509_

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/spawn.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

#include <iostream>
#include <set>
#include <map>
#include <sstream>
#include <stdexcept>

#include "logging.hpp"
#include "network/broadcaster/directory.hpp"
#include "network/client/connection.hpp"
#include "network/protocol.hpp"
#include "processor.hpp"

namespace network {
  namespace client {
    // generic base class for connecting to many broadcasters
    class client_multi_base : public  boost::enable_shared_from_this<client_multi_base>
                            , private boost::noncopyable {
    public:
      typedef boost::shared_ptr<client_multi_base> sptr;
    
      class connection_multi : public connection {
      public:
        typedef boost::shared_ptr<connection_multi> sptr;
        static sptr make(client_multi_base* that,
                         boost::asio::io_service& io_service,
                         std::string name,
                         std::string host,
                         std::string port) {
          return sptr(new connection_multi(that, io_service, name, host, port));
        }

        const std::string& name() const { return name_; }

        virtual void process(data_buffer_type::const_iterator begin,
                             data_buffer_type::const_iterator end) {
          LOG_INFO(str(boost::format("process: h='%s', %s")
                       % header()
                       % name_));
          that_->process(this, begin, end);
        }
      
      protected:
      private:
        connection_multi(client_multi_base* that,
                         boost::asio::io_service& io_service,
                         std::string name,
                         std::string host,
                         std::string port)
          : connection(io_service, host, port)
          , that_(that)
          , name_(name) { }
    
        client_multi_base* that_;
        std::string        name_;
      } ;
  
      //                               (name, streams) -> connection
      typedef std::map<std::pair<std::string, std::string>, connection_multi::sptr> connection_map_type;

      // stream data object holding a copy of the received data + header
      class stream_data {
      public:
        typedef std::vector<char> data_vector_type;
        stream_data() {}
        stream_data(const protocol::header& h,
                    connection::data_buffer_type::const_iterator begin,
                    connection::data_buffer_type::const_iterator end)
          : header_(h) {
          data_.resize(std::distance(begin, end));
          std::copy(begin, end, data_.begin());
        }
        const protocol::header& header() const { return header_; }
        const data_vector_type& data()   const { return data_; }
      protected:
      private:
        protocol::header header_;
        data_vector_type data_;
      } ;

      typedef boost::posix_time::ptime ptime;
      typedef std::multimap<ptime, stream_data > time_data_type;
      typedef std::map<std::string, time_data_type >   name_time_data_type;
      typedef std::map<std::pair<ptime, ptime>, name_time_data_type > history_buffer_type;

      client_multi_base(boost::asio::io_service& io_service,
                        const boost::property_tree::ptree& config)
        : strand_(io_service)
        , mutex_(new boost::mutex)
        , now_(boost::posix_time::not_a_date_time)
        , time_granularity_sec_(config.get<size_t>("ClientMulti.<xmlattr>.time_granularity_sec"))
        , delay_(boost::posix_time::seconds(config.get<size_t>("ClientMulti.<xmlattr>.synchronization_delay_sec"))) {

        // make sure that the time granularity divides the number of seconds in a day
        ASSERT_THROW(((24*60*60) % time_granularity_sec_) == 0);

        // make up the list of connections
        BOOST_FOREACH(const boost::property_tree::ptree::value_type& p, config.get_child("ClientMulti.Servers")) {
          if (p.first == "<xmlattr>") continue;
          ASSERT_THROW(p.first == "Server");
          std::string streams;
          BOOST_FOREACH(const boost::property_tree::ptree::value_type& s, p.second) {
            if (s.first == "<xmlattr>") continue;
            ASSERT_THROW(s.first == "Stream");
            streams += " " + s.second.get<std::string>("<xmlattr>.pattern");
          }
          connect_to(p.second.get<std::string>("<xmlattr>.name"),
                     p.second.get<std::string>("<xmlattr>.host"),
                     p.second.get<std::string>("<xmlattr>.port"),
                     streams);
        }
      }
      virtual ~client_multi_base() {}
  
      void connect_to(std::string name, std::string host, std::string port, std::string streams) {
        connection_multi::sptr c(connection_multi::make(this, io_service(), name, host, port));
        connections_.insert(std::make_pair(std::make_pair(name, streams), c));
      }

      // start all connections
      void start() {
        BOOST_FOREACH(connection_map_type::value_type& cv, connections_) {
          cv.second->start(cv.first.second);
        }
      }
      boost::asio::io_service& io_service() { return strand_.get_io_service(); }  
      boost::asio::strand&     strand()     { return strand_; }  
  
    protected:
      // this method is called by the connected clients to send data
      void process(connection_multi* c,
                   connection::data_buffer_type::const_iterator begin,
                   connection::data_buffer_type::const_iterator end) {
        boost::unique_lock<boost::mutex> lock(*mutex_); // protects the code below

        //  data id : name + stream_name
        const std::string name(c->name()+"_"+c->directory().stream_name_of(c->header().stream_number()));
        const ptime t(c->header().approx_ptime());

        // ignore data which is too old
        if (!history_.empty()) {
          const ptime oldest_ptime(history_.begin()->first.first);
          if (t < oldest_ptime) { // ignore this data as it is too old
            std::cout << "ignoring data: " << t << " < " << oldest_ptime << std::endl;
            return;
          }
        }

        // initialize (first call)
        if (now_ == boost::posix_time::not_a_date_time)
          now_ = t;

        // update the clock
        if (c->header().approx_ptime() > now_)
          now_ = t;

        // make the time interval according to the time granularity
        const long sec_of_day(t.time_of_day().total_seconds());
        const std::pair<ptime, ptime> time_interval
          (std::make_pair
           (ptime(t.date(), boost::posix_time::seconds(time_granularity_sec_*(sec_of_day/time_granularity_sec_))),
            ptime(t.date(), boost::posix_time::seconds(time_granularity_sec_*(sec_of_day/time_granularity_sec_ + 1)))));

        std::cout << "now,t,interval= " << now_ << " "
                  << t << " [" << time_interval.first << "," << time_interval.second << "]" << std::endl;
      
        // insert the data (copy)
        time_data_type& mm(history_[time_interval][name]);
        mm.insert(time_data_type::value_type(t, stream_data(c->header(), begin, end)));

        // process and erase all those intervals which are too old
        const ptime newest_ptime(history_.rbegin()->first.second);
        for (history_buffer_type::iterator i(history_.begin()); i!=history_.end(); ) {
          std::cout << " --- [" << i->first.first << "," << i->first.second
                    << "] newest time: " << newest_ptime
                    << " number of names: " << i->second.size() << std::endl;
          if (newest_ptime - i->first.first > delay_) {
            // process all data for this interval
            std::cout << " --- * processing data for interval= " << i->first.first << "," << i->first.second << std::endl;      
            history_.erase(i++);
          } else {
            ++i;
          }
        }
      }

    private:
      boost::asio::strand              strand_;
      connection_map_type              connections_;          // connection information
      boost::shared_ptr<boost::mutex>  mutex_;                // protectes the process method
      ptime                            now_;                  // current time = newest time of all received data
      history_buffer_type              history_;              // holds data received from servers
      size_t                           time_granularity_sec_; // in seconds
      boost::posix_time::time_duration delay_;                // time delay for synchronization
    } ;

  } // namespace client
} // namespace network
#endif // _CLIENT_MULTI_BASE_HPP_cm140509_
