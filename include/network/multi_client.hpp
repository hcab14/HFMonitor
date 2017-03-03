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
#ifndef _MULTI_CLIENT_HPP_cm121228_
#define _MULTI_CLIENT_HPP_cm121228_

#include <map>
#include <string>

#include <boost/format.hpp>
#include <boost/regex.hpp>
#include <boost/tuple/tuple.hpp>

#include "network.hpp"
#include "network/client/client_base.hpp"
#include "network/client/service_net.hpp"
#include "processor.hpp"
#include "processor/registry.hpp"

namespace network {
  namespace client {

    class multi_client : public client_base {
    public:
      // stream name -> processor sptr
      typedef std::string stream_name;
      typedef std::multimap<stream_name, processor::base::sptr> processor_id_map;

      // (regex matching stream names, processor type, config)
      typedef std::string processor_type;
      typedef std::vector<boost::tuple<boost::regex, processor_type, boost::property_tree::ptree> > processor_type_map;

      // stream_name (can be regular expression) -> processor_name
      typedef std::string processor_name;
      typedef std::multimap<stream_name, processor_name> stream_processor_map;

      // map of results
      typedef std::map<std::string, processor::result_base::sptr> result_map;

      // service_mc
      class service_mc : public processor::service_base {
      public:
        typedef boost::shared_ptr<service_mc> sptr;
        static sptr make(multi_client* pp,
                         processor::service_base::sptr sp) {
          return sptr(new service_mc(pp, sp));
        }
        virtual ~service_mc() {}

        virtual std::string     id()                  const { return sp_->id(); }
        virtual ptime           approx_ptime()        const { return sp_->approx_ptime(); }
        virtual boost::uint16_t stream_number()       const { return sp_->stream_number(); }
        virtual std::string     stream_name()         const { return sp_->stream_name(); }

        virtual void put_result(processor::result_base::sptr rp) {
          pp_->put_result(rp);
        }
        virtual processor::result_base::sptr get_result(std::string name) const {
          return pp_->get_result(name);
        }

      protected:
      private:
        service_mc(multi_client* pp,
                   processor::service_base::sptr sp)
          : pp_(pp)
          , sp_(sp) {}

        multi_client* pp_;
        const processor::service_base::sptr sp_;
      } ;

      //
      multi_client(const boost::property_tree::ptree& config)
        : client_base(network::get_io_service(), config)
        , config_(config) {}
      virtual ~multi_client() {}

      bool connect_to(const stream_processor_map& map) {
        std::ostringstream stream_names;
        for (auto const& p : map)
          stream_names << p.first << " ";
        if (not client_base::connect_to(stream_names.str()))
          return false;
        for (auto const& p : map) 
          add_processors(p.first, p.second);
        return true;
      }
  
      bool connect_to(std::string stream_names,
                      std::string processor_name)  {
        if (client_base::connect_to(stream_names) == false)
          return false;
        add_processors(stream_names, processor_name);
        return true;
      }

      virtual void process(data_buffer_type::const_iterator begin,
                           data_buffer_type::const_iterator end) {

        const stream_name str_name(get_directory().stream_name_of(get_header().stream_number()));
        std::pair<processor_id_map::iterator, processor_id_map::iterator> range(processor_id_map_.equal_range(str_name));
        if (range.first == range.second) {
          return; // TODO: complain
        }

        // make up service object
        processor::service_base::sptr sp(service_mc::make(this, service_net::make(get_header(), get_directory())));
        for (; range.first != range.second; ++range.first)
          range.first->second->process(sp, begin, end);
      }

      virtual processor::result_base::sptr dump(processor::result_base::sptr rp) {
        // to be overwritten in a derived class
        return rp;
      }

      virtual void put_result(processor::result_base::sptr rp) {
        //     LOG_INFO(str(boost::format("put_result: %s %s") % rp->name() % rp->to_string()));
        result_map_[rp->name()] = dump(rp);
      }
      virtual processor::result_base::sptr get_result(std::string name) const {
        result_map::const_iterator i(result_map_.find(name));
        return (i != result_map_.end()) ? i->second : processor::result_base::sptr();
      }

    protected:
      typedef std::pair<processor_type, boost::property_tree::ptree> pp_type;

      boost::property_tree::ptree::value_type find_ptree_for_processor(std::string processor_name) {
        for (auto const& s : config_.get_child("Clients")) {
          if (s.second.get<std::string>("<xmlattr>.proc_name") == processor_name)
            return s;
        }
        throw std::runtime_error(str(boost::format("processor defiinition for '%s' not found") % processor_name));
      }

      // add processors with name processor_name for the streams matcing stream_name
      void add_processors(std::string stream_names, std::string processor_name) {
        const boost::property_tree::ptree::value_type s(find_ptree_for_processor(processor_name));    
        std::istringstream iss(stream_names);
        std::string stream_name;
        while (iss >> stream_name)
          processor_type_map_.push_back(boost::make_tuple(boost::regex(stream_name),
                                                          s.first,
                                                          s.second));
      }

      // notifies derived classes of an update of directory
      virtual void directory_update(const broadcaster::directory& new_directory) {
        // remove processors not present anymore
        for (processor_id_map::iterator i(processor_id_map_.begin()); i!=processor_id_map_.end();) {
          if (not new_directory.contains(i->first))
            processor_id_map_.erase(i++);
          else 
            ++i;
        }
        // construct new processors, if needed
        for (broadcaster::directory::const_iterator i(new_directory.begin()), iend(new_directory.end()); i!=iend; ++i) {
          stream_name str_name(i->first);
          // if not found: make new
          if (processor_id_map_.find(str_name) != processor_id_map_.end()) continue;
          const std::vector<pp_type> ps(find_type_of_stream(str_name));
          // p.first : processor name
          // p.second: to be used path in ptree config
          for (auto const& p : ps) {
            std::ostringstream oss;
            boost::property_tree::write_xml(oss, p.second);
            LOG_INFO(str(boost::format("making processor '%s' type='%s' config='%s' ")
                         % str_name % p.first % oss.str()));
            // when the configuration contains an attribute "type" use this value for constructing the processor
            // (used for FFTProcessor only)
            std::string type_attr(p.second.get<std::string>("<xmlattr>.type", p.first));
            processor_id_map_.insert(std::make_pair(str_name, processor::registry::make(type_attr, p.second)));
          }
        }
      }

      // returns vector of pairs of (processor_type, ptree)
      std::vector<pp_type> find_type_of_stream(std::string stream_name) const {
        std::vector<pp_type> result;
        for (processor_type_map::const_iterator i(processor_type_map_.begin()), end(processor_type_map_.end()); i!=end; ++i) {
          if (regex_match(stream_name, i->get<0>()))
            result.push_back(std::make_pair(i->get<1>(), i->get<2>()));
        }
        return result;
      }

    private:
      const boost::property_tree::ptree config_;   // holds copy of config 

      processor_id_map   processor_id_map_;
      processor_type_map processor_type_map_;
      result_map         result_map_;
    } ;

  } // namespace client
} // namespace network
#endif // _MULTI_CLIENT_HPP_cm121228_
