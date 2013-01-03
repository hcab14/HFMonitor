// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_PROCESSOR_HPP_cm100729_
#define _FFT_PROCESSOR_HPP_cm100729_

#include <vector>
#include <map>
#include <complex>
#include <boost/property_tree/ptree.hpp>
#include <boost/integer.hpp>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>

#include "FFT.hpp"
#include "FFTProcessor/Action.hpp"
#include "FFTProcessor/Proxy.hpp"
#include "FFTProcessor/Result.hpp"
#include "FFTProcessor/Result/Calibration.hpp"
#include "InvertMatrix.hpp"
#include "logging.hpp"
#include "network/protocol.hpp"
#include "processor.hpp"
#include "processor/IQBuffer.hpp"
#include "processor/service.hpp"
#include "Spectrum.hpp"

template<typename FFTFloat>
class FFTProcessor : public processor::base_iq {
public:
  // typedef float FFTFloat;
  typedef std::complex<double> Complex;
  typedef std::vector<Complex> Samples;

  typedef std::string LevelKey;
  typedef class ActionKey {
  public:
    ActionKey(std::string name, size_t number) : name_(name), number_(number) {}
    std::string name() const { return name_; }
    size_t number() const { return number_; }
    friend std::ostream& operator<<(std::ostream& os, const ActionKey& a) {
      return os << "(" << a.name() << ", " << a.number() << ")";
    }
    friend bool operator<(const ActionKey& a1, const ActionKey& a2) {
      return a1.makePair() < a2.makePair();
    }
    std::string toString() const { return name_ + str(boost::format("_%04d")  % number_); }
    std::pair<std::string, size_t> makePair() const { return std::make_pair(name_, number_); }
  private:
    std::string name_;
    size_t number_;
  } ActionKey;
  typedef std::map<ActionKey, Action::Handle> ActionMap;
  typedef std::map<LevelKey, ActionMap> LevelMap;

  typedef std::string ResultKey;
  typedef std::map<ResultKey, Result::Base::Handle> ResultMap;

  class FFTProxy : public Proxy::Base {
  public:
    FFTProxy(::processor::service_iq::sptr sp, std::string level, ResultMap& resultMap)
      : ptime_(sp->approx_ptime())
      , level_(level)
      , resultMap_(resultMap) {}

    virtual void putResult(std::string resultKey, Result::Base::Handle result) {
      // LOG_INFO_T(approxPTime_, str(boost::format("FFTProxy::result [%s] =  %s") % resultKey % result));
      std::string key(level() + "." + resultKey);      
      if (resultMap_.find(key) != resultMap_.end())
        LOG_WARNING_T(ptime_, str(boost::format("overwriting key %s") % key));
      resultMap_[key] = result;
    }

    virtual Result::Base::Handle getResult(std::string keyString) const {
      ResultMap::const_iterator i(resultMap_.find(keyString));
      ASSERT_THROW(i != resultMap_.end());
      return i->second;
    }
    virtual ptime getApproxPTime() const { return ptime_; }
    virtual double volt2dbm(double v) const {
      // P = U*U/R, R=50 Ohm, relative to 1mW
      return 10.*std::log10(0.5*v*v/50.) + 30.; // - 3.*header_.adcPreamp() + 10.*header_.attenId();
    }
    virtual double rms_dbm() const {
      return volt2dbm(1.0/static_cast<double>(1L << 24));
    }
  private:
    std::string level() const { return level_; }
    const ::processor::service_base::ptime ptime_;
    const std::string level_;
    ResultMap& resultMap_;
  } ;
 
  FFTProcessor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , fftw_(1024, FFTW_BACKWARD, FFTW_ESTIMATE)
    , windowFcnName_(config.get<std::string>("<xmlattr>.windowFunction"))
    , calibrationKey_(config.get<std::string>("Actions.<xmlattr>.calibrationKey"))
    , modCounter_(std::max(1u, config.get<unsigned>("<xmlattr>.numberOfCollectedEpochs"))) {
    using boost::property_tree::ptree;
    // Levels
    BOOST_FOREACH(const ptree::value_type& level, config.get_child("Actions")) {
      if (level.first.size() == 0 || level.first[0] == '<') continue;
      LOG_INFO(str(boost::format("Level: %s") % level.first));
      // Actions
      size_t counter(0);
      BOOST_FOREACH(const ptree::value_type& action, level.second) {
        const ActionKey actionKey(action.first, counter++);
        LOG_INFO(str(boost::format(" +--- Action: %s") % actionKey));
        actions_[level.first][actionKey] = Action::Factory::makeAction(actionKey.name(), action.second);
      }
    }
  }
  virtual ~FFTProcessor() {}

  static sptr make(const boost::property_tree::ptree& config) {
    sptr result(new FFTProcessor<FFTFloat>(config));
    return result;
  }

  virtual void process_iq(::processor::service_iq::sptr sp,
                          Samples::const_iterator i0,
                          Samples::const_iterator i1) {
    const size_t length(std::distance(i0, i1));
    if (length != fftw_.size()) fftw_.resize(length);
    LOG_INFO_T(sp->approx_ptime(), str(boost::format("FFTProcessor::procIQ %s") % sp->id()));
    if (windowFcnName_ == "Rectangular")
      fftw_.transformRange(i0, i1, FFT::WindowFunction::Rectangular<Complex::value_type>());
    else if (windowFcnName_ == "Hanning")
      fftw_.transformRange(i0, i1, FFT::WindowFunction::Hanning<Complex::value_type>());
    else if (windowFcnName_ == "Hamming")
      fftw_.transformRange(i0, i1, FFT::WindowFunction::Hamming<Complex::value_type>());
    else if (windowFcnName_ == "Blackman")
      fftw_.transformRange(i0, i1, FFT::WindowFunction::Blackman<Complex::value_type>());
    else 
      throw std::runtime_error(THROW_SITE_INFO(windowFcnName_ + ": unknown window function"));
    
    const FFTWSpectrum<FFTFloat> s(fftw_, sp->sample_rate_Hz(), sp->center_frequency_Hz());
  
    // operate on Spectrum
    ResultMap resultMap;
    // set default calibration from the last epoch
    // it will be overwritten with the new calibration
    if (calibrationHandle_ != 0)
      resultMap[calibrationKey_] = calibrationHandle_;

    BOOST_FOREACH(const typename LevelMap::value_type& level, actions_) {
      BOOST_FOREACH(const typename ActionMap::value_type& action, level.second) {
        FFTProxy proxy(sp, level.first, resultMap);
        // LOG_INFO(str(boost::format("%s %s") % level.first % action.first));
        action.second->perform(proxy, s);
      }
    }

    ++modCounter_;

    // collect results
    BOOST_FOREACH(const ResultMap::value_type& result, resultMap) {
      ResultMap::iterator i(resultBuffer_.find(result.first));
      if (i != resultBuffer_.end())
        i->second->push_back(result.second);
      else
        resultBuffer_[result.first] = result.second;
    }
    
    // output results
    if (modCounter_ == 0) {
      BOOST_FOREACH(const ResultMap::value_type& result, resultBuffer_) {
        dump(result);
        result.second->clear();
      }
      resultBuffer_.clear();
    }

    // check if calibration has succeded
    // store (a copy of) the cal handle with "worst-case" covariance for use in the next epoch
    const ResultMap::iterator i(resultMap.find(calibrationKey_));
    if (i != resultMap.end()) {
      Result::Calibration::Handle
        ch(boost::dynamic_pointer_cast<Result::Calibration>(i->second));
      if (ch != 0)
        calibrationHandle_= ch->withWorstCaseCov();
      else
        LOG_WARNING_T(sp->approx_ptime(),
                      str(boost::format("resultMap[%s] is not of type Result::Calibration::Handle")
                          % calibrationKey_));
    }
  }

protected:
  virtual void dump(const ResultMap::value_type& result) {
    // to be overwritten in a derived class
  }

private:
  FFT::FFTWTransform<FFTFloat> fftw_;
  std::string windowFcnName_;
  std::string calibrationKey_;
  Internal::ModuloCounter<size_t> modCounter_;
  Result::Base::Handle calibrationHandle_;
  LevelMap actions_;
  ResultMap resultBuffer_;
} ;

#endif // _FFT_PROCESSOR_HPP_cm100729_

