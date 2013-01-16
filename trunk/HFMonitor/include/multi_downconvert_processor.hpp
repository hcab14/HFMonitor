// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _MULTI_DOWNCONVERT_PROCESSOR_HPP_cm130115_
#define _MULTI_DOWNCONVERT_PROCESSOR_HPP_cm130115_

#include <complex>
#include <map>
#include <vector>

#include <boost/format.hpp>
#include <boost/integer.hpp>
#include <boost/property_tree/ptree.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>

#include "filter/fir.hpp"
#include "filter/fir/overlap_save.hpp"
#include "logging.hpp"
#include "processor.hpp"

template<typename FFTFloat>
class multi_downconvert_processor : public processor::base_iq {
private:
  class filter_param {
  public:
    filter_param(std::string name="",
                 double cutoff=0,
                 double offset_Hz=0,
                 size_t decim=1)
      : name_(name)
      , cutoff_(cutoff)
      , offset_Hz_(offset_Hz)
      , offset_(0)
      , decim_(decim)
      , handle_(0)
      , initialized_(false) {}

    std::string name() const { return name_; }
    size_t handle() const { return handle_; }
    double cutoff() const { return cutoff_; }
    double offset() const { return offset_; }
    double offset_Hz() const { return offset_Hz_; }
    size_t decim() const { return decim_; }
    bool initialized() const { return initialized_; }

    filter_param& update_offset(double sample_rate_Hz) {
      assert(sample_rate_Hz != 0.0);
      offset_ = offset_Hz()/sample_rate_Hz;
      return *this;
    }
    filter_param& set_initialized(const std::pair<size_t,double>& r,
                                  double sample_rate_Hz) {
      offset_      = r.second;
      offset_Hz_   = offset_ * sample_rate_Hz;
      handle_      = r.first;
      initialized_ = true;
      return *this;
    }

    friend std::ostream& operator<<(std::ostream& os, const filter_param& fp) {
      return os << fp.name() << " " << fp.offset() << " " << fp.offset_Hz() << " " << fp.decim();
    }

  protected:
  private:
    std::string name_;
    double     cutoff_;     // normalized cutoff frequency
    double     offset_Hz_;  // in Hz
    double     offset_;     // normalized frequency
    size_t     decim_;
    size_t     handle_;     // overlap_save handle
    bool       initialized_; //
  } ;

  typedef std::vector<filter_param> filter_params;

public: 
  typedef boost::property_tree::ptree ptree;
  multi_downconvert_processor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , overlap_save_(config.get<size_t>("<xmlattr>.l"),
                    config.get<size_t>("<xmlattr>.m")) {
    // Filters
    BOOST_FOREACH(const ptree::value_type& filt, config.get_child("Filters")) {
      if (filt.first == "FIR") {
        const filter_param fp(filt.second.get<std::string>(""), 
                              filt.second.get<double>("<xmlattr>.cutoff"),
                              filt.second.get<double>("<xmlattr>.centerFrequency_Hz"),
                              filt.second.get<size_t>("<xmlattr>.decim"));
        std::cout << fp << std::endl;
        filter_params_.push_back(fp);
      } else {
        std::cout << "unknown filter type: " << filt.first << std::endl;
      }
    }
  }
  virtual ~multi_downconvert_processor() {}

  static sptr make(const boost::property_tree::ptree& config) {
    sptr result(new multi_downconvert_processor<FFTFloat>(config));
    return result;
  }

  virtual void process_iq(processor::base_iq::service::sptr sp,
                          processor::base_iq::const_iterator i0,
                          processor::base_iq::const_iterator i1) {
    // (1) update filters
    BOOST_FOREACH(filter_param& fp, filter_params_) {
      if (not fp.initialized()) {
        fp.update_offset(sp->sample_rate_Hz());
        typename filter::fir::lowpass<FFTFloat> fir(overlap_save_.p());
        fir.design(fp.cutoff(), fp.cutoff()/5.);        
        fp.set_initialized(overlap_save_.add_filter(fir.coeff(), fp.offset(), fp.decim()),
                           sp->sample_rate_Hz());
      }
    }
    // (2)
    overlap_save_.proc(i0, i1);
    // (3)
    BOOST_FOREACH(filter_param& fp, filter_params_) {
      const typename filter::fir::overlap_save<FFTFloat>::complex_vector_type&
        out(overlap_save_.get_filter(fp.handle())->result());
      std::cout << "out: " << fp.name() << " " << out.size() << std::endl;
    }    
  }

protected:
//   virtual void dump(const ResultMap::value_type& result) {
//     // to be overwritten in a derived class
//   }

private:
  typename filter::fir::overlap_save<FFTFloat> overlap_save_;
  filter_params filter_params_;
} ;

#endif // _MULTI_DOWNCONVERT_PROCESSOR_HPP_cm130115_

