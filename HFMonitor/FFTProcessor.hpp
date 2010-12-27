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

#include "logging.hpp"
#include "InvertMatrix.hpp"

#include "protocol.hpp"
#include "FFT.hpp"
#include "Spectrum.hpp"
#include "FFTProxy.hpp"
#include "FFTResult.hpp"
#include "FFTAction.hpp" 

template<typename FFTFloat>
class FFTProcessor {
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
    FFTProxy(ptime approxPTime, std::string level, ResultMap& resultMap)
      : approxPTime_(approxPTime)
      , level_(level)
      , resultMap_(resultMap) {}

    virtual void putResult(std::string resultKey, Result::Base::Handle result) {
      // LOG_INFO_T(approxPTime_, str(boost::format("FFTProxy::result [%s] =  %s") % resultKey % result));
      std::string key(level() + "." + resultKey);      
      if (resultMap_.find(key) != resultMap_.end())
        LOG_WARNING_T(approxPTime_, str(boost::format("overwriting key %s") % key));
      resultMap_[key] = result;
    }

    virtual Result::Base::Handle getResult(std::string keyString) const {
      ResultMap::const_iterator i(resultMap_.find(keyString));
      ASSERT_THROW(i != resultMap_.end());
      return i->second;
    }
    virtual ptime getApproxPTime() const { return approxPTime_; }

  private:
    std::string level() const { return level_; }
    const ptime approxPTime_;
    const std::string level_;
    ResultMap& resultMap_;
  } ;

  FFTProcessor(const boost::property_tree::ptree& config)
    : fftw_(1024, FFTW_BACKWARD, FFTW_ESTIMATE)
    , counter_(0)
    , windowFcnName_(config.get<std::string>("<xmlattr>.windowFunction"))
    , dataPath_(config.get<std::string>("Data.<xmlattr>.path")) {
    using boost::property_tree::ptree;
    // Levels
    BOOST_FOREACH(const ptree::value_type& level, config.get_child("Actions")) {
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
  ~FFTProcessor() {}
  
  void procIQ(const Header& header, 
              Samples::const_iterator i0,
              Samples::const_iterator i1) {
    const size_t length(std::distance(i0, i1));
    if (length != fftw_.size()) fftw_.resize(length);
    LOG_INFO_T(header.approxPTime(), str(boost::format("FFTProcessor::procIQ %s") % header));
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
    
    const FFTWSpectrum<FFTFloat> s(fftw_, header.sampleRate(), header.ddcCenterFrequency());
  
    // operate on Spectrum
    ResultMap resultMap;
    BOOST_FOREACH(const typename LevelMap::value_type& level, actions_) {
      BOOST_FOREACH(const typename ActionMap::value_type& action, level.second) {
        FFTProxy proxy(header.approxPTime(), level.first, resultMap);
        // LOG_INFO(str(boost::format("%s %s") % level.first % action.first));
        action.second->perform(proxy, s);
      }
    }

    // output of results
    BOOST_FOREACH(const ResultMap::value_type& result, resultMap) {
      // LOG_INFO(str(boost::format("result: %s %s") % result.first % result.second));
      try {
        result.second->dump(dataPath_, result.first, header.approxPTime());
      } catch (const std::exception& e) {
        LOG_WARNING_T(header.approxPTime(), e.what());
      }
    }
  }
protected:
private:
  FFT::FFTWTransform<FFTFloat> fftw_;
  unsigned counter_;
  std::string windowFcnName_;
  std::string dataPath_;
  LevelMap actions_;
} ;

#endif // _FFT_PROCESSOR_HPP_cm100729_

