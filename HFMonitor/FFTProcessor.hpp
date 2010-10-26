// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_PROCESSOR_HPP_cm100729_
#define _FFT_PROCESSOR_HPP_cm100729_

#include <iostream>
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
    FFTProxy(std::string level, ResultMap& resultMap)
      : level_(level)
      , resultMap_(resultMap) {}

    virtual void putResult(std::string resultKey, Result::Base::Handle result) {
      std::cout << "FFTProxy::result [" << resultKey << "] " << result << std::endl;
      std::string key(level() + "." + resultKey);      
      if (resultMap_.find(key) != resultMap_.end())
        std::cout << "Warning: overwriting key "+key << std::endl;
      resultMap_[key] = result;
    }

    virtual Result::Base::Handle getResult(std::string keyString) const {
      ResultMap::const_iterator i(resultMap_.find(keyString));
      if (i != resultMap_.end())
        return i->second;
      else throw std::runtime_error(keyString+": getResult not found");
    }
    
    std::string level() const { return level_; }
  private:
    const std::string level_;
    ResultMap& resultMap_;
  } ;

  FFTProcessor(const boost::property_tree::ptree& config)
    : fftw_(1024, FFTW_BACKWARD, FFTW_ESTIMATE)
    , counter_(0)
    , windowFcnName_(config.get<std::string>("FFT.WindowFunction"))
    , dataPath_(config.get<std::string>("FFT.DataPath")) {
    using boost::property_tree::ptree;
    const ptree& pt(config.get_child("FFT.Actions"));
    // Levels
    for (ptree::const_iterator levelIt(pt.begin()); levelIt!=pt.end(); ++levelIt) {
      LevelKey levelKey(levelIt->first);
      const ptree& levelPt(levelIt->second);
      std::cout << "Level:" << levelKey << std::endl;
      size_t counter(0);
      for (ptree::const_iterator actionIt(levelPt.begin()); actionIt!=levelPt.end(); ++actionIt, ++counter) {
        ActionKey actionKey(actionIt->first, counter);
        const ptree& actionPt(actionIt->second);
        std::cout << " +--- Action: " << actionKey << std::endl;        
        actions_[levelKey][actionKey] = Action::Factory::makeAction(actionKey.name(), actionPt);
      }
    }
  }
  ~FFTProcessor() {}
  
  void procIQ(const Header& header, 
              Samples::const_iterator i0,
              Samples::const_iterator i1) {
    const size_t length(std::distance(i0, i1));
    if (length != fftw_.size()) fftw_.resize(length);
    std::cout << "FFTProcessor::procIQ " << header << std::endl;    
    if (windowFcnName_ == "Rectangular")
      fftw_.transformRange(i0, i1, FFT::WindowFunction::Rectangular<Complex::value_type>());
    else if (windowFcnName_ == "Hanning")
      fftw_.transformRange(i0, i1, FFT::WindowFunction::Hanning<Complex::value_type>());
    else if (windowFcnName_ == "Hamming")
      fftw_.transformRange(i0, i1, FFT::WindowFunction::Hamming<Complex::value_type>());
    else if (windowFcnName_ == "Blackman")
      fftw_.transformRange(i0, i1, FFT::WindowFunction::Blackman<Complex::value_type>());
    else 
      throw std::runtime_error(windowFcnName_ + ": unknown window function");      
    
    const FFTWSpectrum<FFTFloat> s(fftw_, header.sampleRate(), header.ddcCenterFrequency());
  
    // operate on Spectrum
    ResultMap resultMap;
    for (typename LevelMap::const_iterator i(actions_.begin()); i!=actions_.end(); ++i) {
      std::string levelName(i->first);
      const ActionMap& levelActions(i->second);
      for (typename ActionMap::const_iterator j(levelActions.begin()); j!=levelActions.end(); ++j) {
        ActionKey actionKey(j->first);
        FFTProxy proxy(levelName, resultMap);
        std::cout << levelName << " " << actionKey << std::endl;
        j->second->perform(proxy, s);
      }
    }

    // output of results
    for (typename ResultMap::const_iterator i(resultMap.begin()); i!=resultMap.end(); ++i) {
      std::cout << "result: " << i->first << " " << *(i->second) << std::endl;
      i->second->dump(dataPath_, i->first, header.approxPTime());
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

