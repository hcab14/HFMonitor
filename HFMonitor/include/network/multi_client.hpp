// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

#ifndef _MULTI_CLIENT_HPP_cm121228_
#define _MULTI_CLIENT_HPP_cm121228_

#include <boost/regex.hpp>
#include <string>

#include "boost/tuple/tuple.hpp"
#include "network.hpp"
#include "network/client/client_base.hpp"
#include "network/client/service_net.hpp"
#include "processor.hpp"
#include "processor/registry.hpp"

class multi_client : public client_base {
public:
  // map stream name -> processor sptr
  typedef std::string stream_name;
  typedef std::map<stream_name, processor::base::sptr> processor_id_map;

  // regex (matching stream names) -> processor type
  typedef std::string processor_type;
  typedef std::vector<boost::tuple<boost::regex, processor_type, std::string> > processor_type_map;

  // stream_name -> processor_type
  typedef std::map<stream_name, processor_type> stream_processor_map;

  // stream_name -> (processor_type, config_id)
  typedef std::map<stream_name, std::pair<processor_type, std::string> > stream_processor_config_map;

  //
  multi_client(const boost::property_tree::ptree& config)
    : client_base(network::get_io_service(), config)
    , config_(config) {}
  virtual ~multi_client() {}

  bool connect_to(const stream_processor_config_map& map) {
    std::ostringstream stream_names;
    BOOST_FOREACH(const stream_processor_config_map::value_type& p, map)
      stream_names << p.first << " ";
    if (not client_base::connect_to(stream_names.str()))
      return false;
    BOOST_FOREACH(const stream_processor_config_map::value_type& p, map) 
      add_processors(p.first, p.second.first, p.second.second);
    return true;
  }

  bool connect_to(const stream_processor_map& map) {
    std::ostringstream stream_names;
    BOOST_FOREACH(const stream_processor_map::value_type& p, map)
      stream_names << p.first << " ";
    if (not client_base::connect_to(stream_names.str()))
      return false;
    BOOST_FOREACH(const stream_processor_map::value_type& p, map) 
      add_processors(p.first, p.second);
    return true;
  }
  
  bool connect_to(std::string stream_names,
		  std::string processor_type,
		  std::string ptree_path="")  { // extra_argument -> key in ptree
    if (client_base::connect_to(stream_names) == false)
      return false;
    add_processors(stream_names, processor_type, ptree_path);
    return true;
  }

  virtual void process(data_buffer_type::const_iterator begin,
                       data_buffer_type::const_iterator end) {
    const stream_name str_name(get_directory().stream_name_of(get_header().stream_number()));
    processor_id_map::iterator i(processor_id_map_.find(str_name));
    if (i == processor_id_map_.end())
      return; // TODO: complain

    // make up service object
    processor::service_base::sptr sp(service_net::make(get_header(), get_directory()));
    i->second->process(sp, begin, end);
  }

protected:
  void add_processors(std::string stream_names, std::string processor_type, std::string ptree_path="") {
    std::istringstream iss(stream_names);
    std::string stream_name;
    while (iss >> stream_name)
      processor_type_map_.push_back(boost::make_tuple(boost::regex(stream_name),
                                                      processor_type,
                                                      ptree_path));
  }

  // notifies derived classes of an update of directory
  virtual void directory_update(const broadcaster_directory& new_directory) {
    // remove processors not present anymore
    for (processor_id_map::iterator i(processor_id_map_.begin()); i!=processor_id_map_.end();) {
      if (not new_directory.contains(i->first))
        processor_id_map_.erase(i++);
      else 
        ++i;
    }
    // construct new processors, if needed
    for (broadcaster_directory::const_iterator i(new_directory.begin());
         i!=new_directory.end(); ++i) {
      const stream_name str_name(i->first);
      // if not found: make new
      if (processor_id_map_.find(str_name) != processor_id_map_.end()) continue;
      try { // find_type_of_stream(...) may throw an exception
        const std::pair<processor_type, std::string> p(find_type_of_stream(str_name));
        // p.first : processor name
        // p.second: to be used path in ptree config
        processor_id_map_[str_name] = processor::registry::make(p.first, config_.get_child(p.second));
      } catch (...) {
        // NOP: 
      }
    }
  }

  // 
  std::pair<processor_type, std::string> find_type_of_stream(std::string stream_name) const {
    processor_type_map::const_iterator end(processor_type_map_.end());
    for (processor_type_map::const_iterator i(processor_type_map_.begin()); i!=end; ++i) {
      if (regex_match(stream_name, i->get<0>()))
        return std::make_pair(i->get<1>(), i->get<2>());
    }
    throw std::runtime_error("stream not found");
  }

private:
  const boost::property_tree::ptree config_;   // holds copy of config 

  processor_id_map   processor_id_map_;
  processor_type_map processor_type_map_;
} ;

#endif // _MULTI_CLIENT_HPP_cm121228_
