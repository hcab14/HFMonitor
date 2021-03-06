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

#include "FFT.hpp"
#include "FFTProcessor/Action.hpp"
#include "FFTProcessor/Proxy.hpp"
#include "FFTProcessor/Result.hpp"
#include "FFTProcessor/Result/Calibration.hpp"
#include "InvertMatrix.hpp"
#include "logging.hpp"
#include "network/protocol.hpp"
#include "processor.hpp"
#include "processor/service.hpp"
#include "Spectrum.hpp"

#include "FFTProcessor/FFTActionFactory.hpp"

/*! \addtogroup processors
 *  @{
 * \addtogroup FFTProcessor FFTProcessor
 * FFT Processor
 *
 * @{
 */

/// FFT processor
template<typename FFTFloat>
class FFTProcessor : public processor::base_iq {
public:
  // typedef float FFTFloat;
  typedef std::complex<float> complex_type;
  typedef std::vector<complex_type> Samples;

#ifdef USE_CUDA
  typedef FFT::CUFFTTransform fft_type;
#else
  typedef typename FFT::FFTWTransform<FFTFloat> fft_type;
#endif

  typedef FFTSpectrum<fft_type> fft_spec_type;

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
  typedef std::map<ActionKey, typename Action::Base<fft_spec_type>::Handle> ActionMap;
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
      result->set_name(key);
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
    , numCollectedEpochs_(std::max(1u, config.get<unsigned>("<xmlattr>.numberOfCollectedEpochs")))
    , epochCounter_(0) {
    using boost::property_tree::ptree;
    // Levels
    for (auto const& level : config.get_child("Actions")) {
      if (level.first.size() == 0 || level.first[0] == '<') continue;
      LOG_INFO(str(boost::format("Level: %s") % level.first));
      // Actions
      size_t counter(0);
      for (auto const& action : level.second) {
        const ActionKey actionKey(action.first, counter++);
        LOG_INFO(str(boost::format(" +--- Action: %s") % actionKey));
        actions_[level.first][actionKey] = Action::Factory::makeAction<fft_spec_type>(actionKey.name(), action.second);
      }
    }
  }
  virtual ~FFTProcessor() {}

  static sptr make(const boost::property_tree::ptree& config) {
    sptr result(new FFTProcessor<FFTFloat>(config));
    return result;
  }

  virtual void process_iq(service::sptr sp,
                          const_iterator i0,
                          const_iterator i1) {
    const size_t length(std::distance(i0, i1));
    if (length != fftw_.size()) fftw_.resize(length);
    LOG_INFO_T(sp->approx_ptime(), str(boost::format("FFTProcessor::procIQ %s") % sp->id()));
    if (windowFcnName_ == "Rectangular")
      fftw_.transformRange(i0, i1, FFT::WindowFunction::Rectangular<complex_type::value_type>(length));
    else if (windowFcnName_ == "Hanning")
      fftw_.transformRange(i0, i1, FFT::WindowFunction::Hanning<complex_type::value_type>(length));
    else if (windowFcnName_ == "Hamming")
      fftw_.transformRange(i0, i1, FFT::WindowFunction::Hamming<complex_type::value_type>(length));
    else if (windowFcnName_ == "Blackman")
      fftw_.transformRange(i0, i1, FFT::WindowFunction::Blackman<complex_type::value_type>(length));
    else
      throw std::runtime_error(THROW_SITE_INFO(windowFcnName_ + ": unknown window function"));

    const FFTSpectrum<fft_type> s(fftw_, sp->sample_rate_Hz(), sp->center_frequency_Hz());

    // operate on Spectrum
    ResultMap resultMap;
    // set default calibration from the last epoch
    // it will be overwritten with the new calibration
    if (calibrationHandle_) {
      calibrationHandle_->updateTime(sp->approx_ptime());
      resultMap[calibrationKey_] = calibrationHandle_;
    } else {
      resultMap[calibrationKey_] = Result::Calibration::makeDefault(sp->approx_ptime(), calibrationKey_);
    }

    for (auto const& level : actions_) {
      for (auto const& action : level.second) {
        FFTProxy proxy(sp, level.first, resultMap);
        // LOG_INFO(str(boost::format("%s %s") % level.first % action.first));
        action.second->perform(proxy, s);
      }
    }

    ++epochCounter_;

    // collect results
    for (auto const& result : resultMap) {
      sp->put_result(result.second);
      auto const i(resultBuffer_.find(result.first));
      if (i != resultBuffer_.end())
        i->second->push_back(result.second);
      else
        resultBuffer_[result.first] = result.second;
    }

    // output results
    if (epochCounter_ == numCollectedEpochs_) {
      epochCounter_ = 0;
      for (auto const& result : resultBuffer_) {
        dump(result);
        result.second->clear();
      }
      resultBuffer_.clear();
    }

    // check if calibration has succeded
    // store (a copy of) the cal handle with "worst-case" covariance for use in the next epoch
    auto const i(resultMap.find(calibrationKey_));
    if (i != resultMap.end()) {
      auto const ch(boost::dynamic_pointer_cast<Result::Calibration>(i->second));
      if (ch)
        calibrationHandle_= ch->withWorstCaseCov(calibrationKey_);
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
  fft_type    fftw_;
  std::string windowFcnName_;
  std::string calibrationKey_;
  size_t      numCollectedEpochs_;
  size_t      epochCounter_;
  Result::Base::Handle            calibrationHandle_;
  LevelMap  actions_;
  ResultMap resultBuffer_;
} ;
/// @}
/// @}
#endif // _FFT_PROCESSOR_HPP_cm100729_
