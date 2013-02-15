// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _MULTI_DOWNCONVERT_PROCESSOR_HPP_cm130115_
#define _MULTI_DOWNCONVERT_PROCESSOR_HPP_cm130115_

#include <complex>
#include <map>
#include <sstream>
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
#include "processor/registry.hpp"
#include "tracking_goertzel_processor.hpp"

template<typename FFTFloat>
class multi_downconvert_processor : public processor::base_iq {
protected:
  class filter_param {
  public:
    filter_param(std::string name="",
                 double cutoff=0,
                 double center_freq_Hz=0,
                 size_t decim=1)
      : name_(name)
      , cutoff_(cutoff)
      , center_freq_input_Hz_(0)
      , center_freq_Hz_(center_freq_Hz)
      , offset_(0)
      , decim_(decim)
      , handle_(0)
      , initialized_(false) {}

    std::string name() const { return name_; }
    size_t handle()    const { return handle_; }
    double cutoff()    const { return cutoff_; }
    double center_freq_input_Hz() const { return center_freq_input_Hz_; }
    double center_freq_Hz()       const { return center_freq_Hz_; }
    double offset()    const { return offset_; }
    size_t decim()     const { return decim_; }
    bool initialized() const { return initialized_; }

    filter_param& update_offset(double _center_freq_input_Hz,
                                boost::uint32_t sample_rate_Hz) {
      assert(sample_rate_Hz != 0.0);
      center_freq_input_Hz_ = _center_freq_input_Hz;
      offset_               = (center_freq_input_Hz()-center_freq_Hz())/sample_rate_Hz;
      return *this;
    }
    //                                           (handle, offset)
    filter_param& set_initialized(const std::pair<size_t,double>& r,
                                  boost::uint32_t sample_rate_Hz) {
      std::cout << "set_initialized: offset=" <<std::setprecision(15) << r.second
                << " " << sample_rate_Hz*offset()
                << " " << center_freq_input_Hz() 
                << " " << center_freq_input_Hz() - sample_rate_Hz*offset()
                << std::endl;
      offset_         = r.second;
      center_freq_Hz_ = center_freq_input_Hz() - sample_rate_Hz*offset();
      handle_         = r.first;
      initialized_    = true;
      return *this;
    }

    friend std::ostream& operator<<(std::ostream& os, const filter_param& fp) {
      return os << fp.name() << " " << fp.offset() << " " << fp.center_freq_Hz() << " " << fp.decim();
    }

  protected:
  private:
    std::string name_;                 // filter name
    double      cutoff_;               // normalized cutoff frequency
    double      center_freq_input_Hz_; //
    double      center_freq_Hz_;       // 
    double      offset_;               // normalized frequency (relative to center_freq_input_Hz)
    size_t      decim_;                // decimation factor
    size_t      handle_;               // overlap_save handle
    bool        initialized_;          //
  } ;

  // service_iq
  class service_dc : public service {
  public:
    typedef boost::shared_ptr<service_dc> sptr;
    static sptr make(multi_downconvert_processor<FFTFloat>* pp,
                     service::sptr   sp,
                     std::string     stream_name,
                     boost::uint32_t sample_rate_Hz,
                     double          center_frequency_Hz) {
      return sptr(new service_dc(pp, sp, stream_name,
                                 sample_rate_Hz, center_frequency_Hz));
    }
    virtual ~service_dc() {}

    virtual std::string     id()                  const { return sp_->id(); }
    virtual ptime           approx_ptime()        const { return sp_->approx_ptime(); }
    virtual boost::uint16_t stream_number()       const { return sp_->stream_number(); }
    virtual std::string     stream_name()         const { return stream_name_; }
    virtual boost::uint32_t sample_rate_Hz()      const { return sample_rate_Hz_; }
    virtual double          center_frequency_Hz() const { return center_frequency_Hz_; }
    virtual float           offset_ppb()          const { return offset_ppb_.first; }
    virtual float           offset_ppb_rms()      const { return offset_ppb_.second; }

    virtual void put_result(processor::result_base::sptr rp) {
      pp_->put_result(rp);
    }
    virtual processor::result_base::sptr get_result(std::string name) const {
      return pp_->get_result(name);
    }

  protected:
  private:
    service_dc(multi_downconvert_processor<FFTFloat>* pp,
               service::sptr   sp,
               std::string     stream_name,
               boost::uint32_t sample_rate_Hz,
               double          center_frequency_Hz)
      : pp_(pp)
      , sp_(sp)
      , stream_name_(stream_name)
      , sample_rate_Hz_(sample_rate_Hz)
      , center_frequency_Hz_(center_frequency_Hz)
      , offset_ppb_(pp->calibration_offset_ppb()) {
      LOG_INFO(str(boost::format("service_dc: calibration_offset_ppb: %.3f +- %.3f")
                   % offset_ppb_.first
                   % offset_ppb_.second));
    }

    multi_downconvert_processor<FFTFloat>* pp_;
    const service::sptr   sp_;
    const std::string     stream_name_;
    const boost::uint32_t sample_rate_Hz_;
    const double          center_frequency_Hz_;
    const std::pair<double,double> offset_ppb_;
  } ;

  // list of filter parameters
  typedef std::vector<filter_param> filter_params;

  // filter name -> associated iq processor sptr
  typedef std::map<std::string, processor::base_iq::sptr> processor_map;

  // 
  typedef std::map<std::string, processor::result_base::sptr> result_map;

  // 
  typedef std::vector<std::string> calibration_keys;

public: 
  typedef boost::shared_ptr< multi_downconvert_processor<FFTFloat> > sptr;
  typedef typename filter::fir::overlap_save<FFTFloat> overlap_save_type;

  multi_downconvert_processor(const ptree& config)
    : base_iq(config)
    , overlap_save_(config.get<size_t>("<xmlattr>.l"),
                    config.get<size_t>("<xmlattr>.m")) {
    // Filters
    BOOST_FOREACH(const ptree::value_type& filt, config.get_child("Filters")) {
      if (filt.first == "FIR") {
        const filter_param fp(filt.second.get<std::string>("<xmlattr>.name"), 
                              filt.second.get<double>("<xmlattr>.cutoff"),
                              filt.second.get<double>("<xmlattr>.centerFrequency_Hz"),
                              filt.second.get<size_t>("<xmlattr>.decim"));
        filter_params_.push_back(fp);
        LOG_INFO(str(boost::format("multi_downconvert_processor: filter %s") % fp));
      } else {
        LOG_ERROR(str(boost::format("multi_downconvert_processor: unknown filter type '%s'")
                      % filt.first));
      }
    }    
    // Processors
    BOOST_FOREACH(const ptree::value_type& p, config.get_child("Processors")) {
      const std::string name(p.second.get<std::string>("<xmlattr>.name"));
      const std::string input(p.second.get<std::string>("<xmlattr>.input"));
      processor::base_iq::sptr
        pp(boost::dynamic_pointer_cast<processor::base_iq>(processor::registry::make(p.first, p.second)));
      if (not pp) {
        LOG_ERROR(str(boost::format("failed to make processor '%s' type='%s'") % name % p.first));
        continue;
      }
      LOG_INFO(str(boost::format("making processor '%s' type='%s' input='%s'") % name % p.first % input));
      processor_map_[input] = pp;
    }
    // Calibration
    const std::string cal_algo(config.get_child("Calibration").get<std::string>("<xmlattr>.algorithm"));
    if (cal_algo != "WeightedMean")
      LOG_ERROR(str(boost::format("unsupported calibration algorithm '%s' requested") % cal_algo));
    BOOST_FOREACH(const ptree::value_type& p, config.get_child("Calibration")) { 
      if (p.first != "Key") {
        LOG_ERROR(str(boost::format("ignoring Calibration tag '%s'") % p.first));
        continue;
      }
      const std::string cal_key(p.second.get<std::string>(""));
      LOG_INFO(str(boost::format("adding calibration key '%s'") % cal_key));
      calibration_keys_.push_back(cal_key);
    }
  }

  virtual ~multi_downconvert_processor() {}

  static sptr make(const ptree& config) {
    sptr result(new multi_downconvert_processor<FFTFloat>(config));
    return result;
  }

  virtual void process_iq(service::sptr sp,
                          const_iterator i0,
                          const_iterator i1) {
    // (1) initialize filters
    BOOST_FOREACH(filter_param& fp, filter_params_) {
      if (not fp.initialized()) {
        fp.update_offset(sp->center_frequency_Hz(), sp->sample_rate_Hz());
        typename filter::fir::lowpass<FFTFloat> fir(overlap_save_.p());
        fir.design(fp.cutoff(), fp.cutoff()/5.);        
        fp.set_initialized(overlap_save_.add_filter(fir.coeff(), fp.offset(), fp.decim()),
                           sp->sample_rate_Hz());
      }
    }

    // (2) process I/Q samples
    overlap_save_.proc(i0, i1);

    // (3) dump the output of all filters
    BOOST_FOREACH(const filter_param& fp, filter_params_) {
      dump(sp, fp, overlap_save_.get_filter(fp.handle())->result());
      processor_map::iterator i(processor_map_.find(fp.name()));
      if (i != processor_map_.end()) {
        LOG_INFO(str(boost::format("proc %s") % fp));
        proc(service_dc::make(this,
                              sp,
                              fp.name(),
                              sp->sample_rate_Hz()/fp.decim(),
                              fp.center_freq_Hz()),
             i->second, overlap_save_.get_filter(fp.handle())->result());
      }
    }
  }

protected:
  // may be overwritten in a derived class
  virtual void dump(service::sptr sp,
                    const filter_param& fp,
                    const typename overlap_save_type::complex_vector_type& out) {
    std::cout << "out: " << fp.name() << " " << out.size() << std::endl;
  }

  virtual void proc(service::sptr sp,
                    processor::base_iq::sptr pp,
                    const typename overlap_save_type::complex_vector_type& out) {
    pp->process_iq(sp, out.begin(), out.end());
  }
  
private:
  // results
  void put_result(processor::result_base::sptr rp) {
    LOG_INFO(str(boost::format("multi_downconvert_processor::put_result %s") % rp->to_string()));
    result_map_[rp->name()] = rp;
  }
  processor::result_base::sptr get_result(std::string name) const {
    result_map::const_iterator i(result_map_.find(name));
    return (i != result_map_.end()) ? i->second : processor::result_base::sptr();
  }

  //        (ppb, ppb_rms)
  std::pair<double, double> calibration_offset_ppb() const {
    double sum_wx(0), sum_w(0);
    BOOST_FOREACH(const std::string& key, calibration_keys_) {
      tracking_goertzel_processor::result::sptr
        rp(boost::dynamic_pointer_cast<tracking_goertzel_processor::result>(get_result(key)));
      if (not rp) {
        LOG_ERROR(str(boost::format("calibration_offset_ppb: key '%s' not found") % rp));
        continue;
      }
      if (not rp->f_Hz().valid()) continue;

      const double rms(1e9*rp->f_Hz().rms_value()/rp->f0_Hz());
      // w_i = 1/sigma_i^2
      const double weight((rms != 0. ? 1./(rms*rms) : 0.));
      sum_w  += weight;
      sum_wx += weight * 1e9*rp->f_Hz().value()/rp->f0_Hz();
    }
    return (sum_w != 0.)
      ? std::make_pair(sum_wx/sum_w, 1./sqrt(sum_w))
      : std::make_pair(0., 0.);
  }

  overlap_save_type overlap_save_;
  filter_params     filter_params_;
  processor_map     processor_map_;
  result_map        result_map_;
  calibration_keys  calibration_keys_;
} ;

#endif // _MULTI_DOWNCONVERT_PROCESSOR_HPP_cm130115_
