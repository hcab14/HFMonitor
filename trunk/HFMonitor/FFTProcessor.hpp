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
#include <boost/filesystem/fstream.hpp>

#include <boost/shared_ptr.hpp>

#include "InvertMatrix.hpp"

#include "protocol.hpp"
#include "FFT.hpp"

class SpectrumBase {
public:
  SpectrumBase(double sampleRate,
               double centerFrequency)
    : sampleRate_(sampleRate)
    , centerFrequency_(centerFrequency) {}

  typedef std::complex<double> Complex;

  double sampleRate() const { return sampleRate_; }
  double centerFrequency() const { return centerFrequency_; }

  virtual size_t freq2Index(double qrg_Hz) const = 0;  // get the nearest bin index
  virtual double index2Freq(size_t index)  const = 0;  // ...
  virtual size_t size()                    const = 0;  // ...
  virtual Complex operator[](size_t index) const = 0;  // ...

private:
  double sampleRate_;
  double centerFrequency_;
} ;

template<typename T>
class Spectrum : public SpectrumBase {
public:
  Spectrum(const FFT::FFTWTransform<T>& fftw,
           double sampleRate,
           double centerFrequency)
    : SpectrumBase(sampleRate, centerFrequency)
    , fftw_(fftw) {}

  virtual size_t freq2Index(double qrg_Hz) const { // get the nearest bin index
    const int n(size());
    const int i(round(double(n)*(qrg_Hz - centerFrequency()) / sampleRate()));
    if (i >= -n/2 && i < n/2)
      return (i>=0) ? i : n+i;
    else 
      throw std::runtime_error("freq2Index failed");
  }

  virtual double index2Freq(size_t index) const { //
    const int n(size());
    return centerFrequency() + sampleRate() * (int(index)>=n/2 ? -n+int(index) : int(index)) / double(n);
  }
  
  virtual size_t size() const { return fftw_.size(); }
  virtual std::complex<double> operator[](size_t index) const { return fftw_.getBin(index); }
  
protected:
private:
  static int round(double d) { return int(d+((d>=0.0) ? .5 : -.5)); }

  const FFT::FFTWTransform<T>& fftw_;
  double sampleRate_;
  double centerFrequency_;
} ;

class FreqStrength {
public:
  FreqStrength(double freq, double strength)
    : freq_(freq)
    , strength_(strength) {}
  
  double freq() const { return freq_; }
  double strength() const { return strength_; }
  double& strength() { return strength_; }
  
  static bool cmpStrength(const FreqStrength& fs1, 
                          const FreqStrength& fs2) {
    return fs1.strength() < fs2.strength();
  }
private:
  double freq_;
  double strength_;
} ;

namespace Result {
  class Base : private boost::noncopyable {
  public:
    typedef boost::shared_ptr<Base> Handle;

    Base(std::string name) 
      : name_(name) {}
    virtual ~Base() {}
    virtual std::string toString() const { return name(); }
    std::string name() const { return name_; }
    // void setName(std::string name) { name_= name; }
    friend std::ostream& operator<<(std::ostream& os, const Base& b) {
      return os << b.toString();
    }
    virtual void dump(std::string path) {}
  protected:
    std::string name_;
  } ;

  class SpectrumPeak : public Base {
  public:
    typedef boost::shared_ptr<SpectrumPeak> Handle;
    SpectrumPeak(double fReference)
      : Base("SpectrumPeak")
      , fReference_(fReference) 
      , fMeasured_(0.) 
      , fMeasuredRMS_(1.) 
      , strength_(0.) 
      , strengthRMS_(1.)
      , ratio_(1.) {}

    bool findPeak(const std::vector<FreqStrength>& fs, double minRatio) {
      std::vector<FreqStrength>::const_iterator
        iMin(std::min_element(fs.begin(), fs.end(), FreqStrength::cmpStrength));
      std::vector<FreqStrength>::const_iterator
        iMax(std::max_element(fs.begin(), fs.end(), FreqStrength::cmpStrength));
      
      const size_t indexMax(std::distance(fs.begin(), iMax));

      double sum(0.0);
      double weight(0.0);
      for (size_t u(indexMax-std::min(indexMax, size_t(3))); u<(indexMax+4) && u < fs.size(); ++u) {
        weight += fs[u].strength();
        sum += fs[u].strength() * fs[u].freq();
      }
      ratio_ = (iMin->strength() != 0.0) ? iMax->strength() / iMin->strength() : 1.0;
      if (ratio_ < minRatio || weight == 0.0) {
        // throw std::runtime_error("ratio < minRatio || weight == 0.0");
        std::cout << "ratio < minRatio || weight == 0.0" << std::endl;
        return false;
      }
      // TODO: error propagation
      const std::pair<double, double> c(cal(sum/weight));
      fMeasured_    = c.first;
      fMeasuredRMS_ = c.second;
      strength_     = iMax->strength();
      strengthRMS_  = iMax->strength()/10.; // TODO
      return true;
    }

    virtual ~SpectrumPeak() {}
    virtual std::string toString() const { 
      std::stringstream ss; 
      ss << Base::toString() 
         << " fReference="   << fReference()
         << " fMeasured="    << fMeasured()
         << " fMeasuredRMS=" << fMeasuredRMS()
         << " strength="     << strength()
         << " strengthRMS="  << strengthRMS()
         << " ratio="        << ratio();
      return ss.str();
    }
    double fReference() const { return fReference_; }
    double fMeasured() const { return fMeasured_; }
    double fMeasuredRMS() const { return fMeasuredRMS_; }
    double strength() const { return strength_; }
    double strengthRMS() const { return strengthRMS_; }
    double ratio() const { return ratio_; }

    // this method is overwritten e.g. in CalibratedSpectrumPeak
    virtual std::pair<double, double> cal(double f) const {
      return std::make_pair(f, double(1));
    }

  private:
    double fReference_;
    double fMeasured_;
    double fMeasuredRMS_;
    double strength_;
    double strengthRMS_;
    double ratio_;
  } ;

  class Calibration : public Base {
  public:
    typedef boost::shared_ptr<Calibration> Handle;
    typedef boost::numeric::ublas::matrix<double> Matrix;
    typedef boost::numeric::ublas::vector<double> Vector;

    Calibration(const std::vector<Result::SpectrumPeak::Handle>& peaks,
                size_t n=2)
      : Base("Calibration")
      , n_(n)
      , q_(n,n)
      , x_(n) {
      using namespace boost::numeric::ublas;
      Matrix a(peaks.size(), n);
      Vector y(peaks.size());
      for (size_t u(0); u<peaks.size(); ++u) {
        y(u)   = peaks[u]->fMeasured();
        double t(1.0);
        for (size_t v(0); v<n; ++v) {
          a(u,v) = t; t *= peaks[u]->fReference();
        }
      }
      // least squares inversion
      Matrix ata(prod(trans(a),a));
      if (ublas_util::InvertMatrix(ata, q_)) {
        x_ = prod(q_, Vector(prod(trans(a),y)));
      } else 
        throw std::runtime_error("Calibration has failed");
    }
    virtual ~Calibration() {}
    virtual std::string toString() const {
      std::stringstream ss;
      ss << Base::toString()
         << " x=" << x()
         << " q=" << q();
      return ss.str();
    }    
    const Vector& x() const { return x_; } // polynomial coefficients
    const Matrix& q() const { return q_; } // variance-covariance matrix for x
    
    // returns a value,RMS pair
    std::pair<double, double> cal2uncal(double fCal) const {
      using namespace boost::numeric::ublas;
      Vector a(n_);
      double t(1.0);
      for (unsigned u(0); u<n_; ++u) {
        a(u)=t; t *= fCal;
      }
      return std::make_pair(inner_prod(a,x_),
                            std::sqrt(inner_prod(a, Vector(prod(q_, a)))));
    }
    // returns a value,RMS pair
    // NOTE: for n_ != 2 this is broken
    std::pair<double,double> uncal2cal(double fUncal) const {
      using namespace boost::numeric::ublas;
      const double fCal((fUncal-x_(0))/x_(1));
      Vector a(2); a(0)=-1.0/x_(1); a(1)=-fCal/x_(1); 
      return std::make_pair(fCal, std::sqrt(inner_prod(a, Vector(prod(q_, a)))));       
    }
    
  private:
    size_t n_;
    Matrix q_;
    Vector x_;
  } ;

  class CalibratedSpectrumPeak : public SpectrumPeak {
  public:
    typedef boost::shared_ptr<SpectrumPeak> Handle;
    CalibratedSpectrumPeak(double fReference,
                           Calibration::Handle calibrationHandle)
      : SpectrumPeak(fReference)
      , calibrationHandle_(calibrationHandle) {
      name_= "CalibratedSpectrumPeak";
    }
    virtual ~CalibratedSpectrumPeak() {}

    virtual std::string toString() const { 
      std::stringstream ss; 
      ss << SpectrumPeak::toString()
         << " diff="   << fMeasured()-fReference();
      return ss.str();
    }

    virtual std::pair<double, double> cal(double f) const {
      return calibrationHandle_->uncal2cal(f);
    }
  private:
    Calibration::Handle calibrationHandle_;
  } ;
} // namespace Result

namespace Proxy {
  class Base {
  public:
    virtual ~Base() {}
    virtual void result(std::string resultKey, Result::Base::Handle result) {
      std::cout << "Proxy::Base::result [" << resultKey << "] " << result << std::endl;
    }
    virtual Result::Base::Handle getResult(std::string keyString) const = 0;
  } ;
  
  typedef boost::shared_ptr<Base> Handle;
}

namespace Action {
  class Base : private boost::noncopyable {
  public:
    Base(std::string name)
      : name_(name) {}
    virtual ~Base() {}
    std::string name() const { return name_; }
    virtual void perform(Proxy::Base& p, const SpectrumBase& s) = 0;
  protected:
    std::string name_;
  } ;

  class SpectumInterval : public Base {
  public:
    SpectumInterval(const boost::property_tree::ptree& config)
      : Base("SpectumInterval")
      , fMin_(config.get<double>("fMin"))
      , fMax_(config.get<double>("fMax"))
      , filterType_(config.find("Filter") != config.not_found() 
                    ? config.get<std::string>("Filter.Type")
                    : "None")
      , filterTimeConstant_((filterType_ != "None") 
                            ? config.get<double>("Filter.TimeConstant")
                            : 1.0) {}
    
    virtual void perform(Proxy::Base& p, const SpectrumBase& s) {
      try {
        const size_t i0(s.freq2Index(fMin_));
        const size_t i1(s.freq2Index(fMax_));
      if (filterType_ == "None") {
        fs_.clear();
        for (size_t u=i0; u<=i1; ++u) 
          fs_.push_back(FreqStrength(s.index2Freq(u), std::abs(s[u])));
      } else if (filterType_ == "LowPass") {
        if (fs_.empty()) {
          for (size_t u=i0; u<=i1; ++u) 
            fs_.push_back(FreqStrength(s.index2Freq(u), std::abs(s[u])));
        } else {
          const double dt(s.size()/s.sampleRate());
          const double x(dt / filterTimeConstant_);
          for (size_t u=i0; u<=i1; ++u) 
            fs_[u-i0].strength() = (1-x) * fs_[u-i0].strength() + x * std::abs(s[u]);
        }
      } else {
        throw std::runtime_error(filterType_ + " unknown filter");
      }
      // call virtual method
      proc(p, s, fs_);      
      } catch (...) {
        return;
      }
    }

    // this method is overwritten by, e.g., FindPeak, see below
    virtual void proc(Proxy::Base& p, 
                      const SpectrumBase& s,
                      const std::vector<FreqStrength>& fs) {}
    
  protected:
  private:
    const double fMin_;
    const double fMax_;
    const std::string filterType_;
    const double filterTimeConstant_;
    std::vector<FreqStrength> fs_;
  } ;

  class FindPeak : public SpectumInterval {
  public:
    FindPeak(const boost::property_tree::ptree& config)
      : SpectumInterval(config)
      , fReference_(config.get<double>("fRef"))
      , minRatio_(config.get<double>("minRatio"))
      , resultKey_(config.get<std::string>("Name"))
      , useCalibration_(false)
      , calibrationKey_("") {
      name_ = "FindPeak";
      if (config.find("Calibration") != config.not_found()) {
        useCalibration_= true;
        calibrationKey_= config.get<std::string>("Calibration");
      }
    }
    
    virtual void proc(Proxy::Base& p, 
                      const SpectrumBase& s,
                      const std::vector<FreqStrength>& fs) {
      std::cout << "FindPeak::perform " << std::endl;
      try {
        Result::SpectrumPeak* 
          spp((useCalibration_) 
              ? new Result::CalibratedSpectrumPeak(fReference_,
                                                   boost::dynamic_pointer_cast<Result::Calibration>
                                                   (p.getResult(calibrationKey_)))
              : new Result::SpectrumPeak(fReference_));
        if (spp->findPeak(fs, minRatio_)) {
          Result::Base::Handle rh();
          p.result(resultKey_, Result::Base::Handle(spp));
        }
      } catch (const std::runtime_error& e) {
        std::cout << e.what() << std::endl;
      }
    }
  private:
    const double fReference_;         // nominal frequency / Hz
    const double minRatio_;           // min. ratio peak/background
    std::string resultKey_;           // result key name
    bool useCalibration_;             // 
    std::string calibrationKey_;      // key name for calibration information
  } ;

  class Calibrator : public Base {
  public:
    Calibrator(const boost::property_tree::ptree& config)
      : Base("Calibrator")
      , resultKey_(config.get<std::string>("Name")) {
      using boost::property_tree::ptree;
      const ptree& pt(config.get_child("Inputs"));
      for (ptree::const_iterator i(pt.begin()); i!=pt.end(); ++i) {
        if (i->first == "Input") {
          std::cout << "Calibrator::Calibrator Input." << i->second.get<std::string>("") << std::endl;
          inputs_.push_back(i->second.get<std::string>(""));
        } else {
          std::cout << "Calibrator::calibrate unknown field " << i->first << std::endl;
        }
      }
    }

    virtual void perform(Proxy::Base& p, const SpectrumBase& s) {
      std::cout << "Calibrator::perform" << std::endl;
      // count data
      std::vector<Result::SpectrumPeak::Handle> peaks;
      for (std::vector<std::string>::const_iterator i(inputs_.begin()); i!=inputs_.end(); ++i) {
        try {
          Result::SpectrumPeak::Handle sp(boost::dynamic_pointer_cast<Result::SpectrumPeak>(p.getResult(*i)));
          if (sp != 0) {
            peaks.push_back(sp);
          }
        } catch (...) {
          // TODO ...
        }
      }
      try {     
        Result::Base::Handle rh(new Result::Calibration(peaks));
        p.result(resultKey_, rh); 
      } catch (...) {
        // ...
      }
    }
  private:
    std::string resultKey_;           // result key name
    std::vector<std::string> inputs_; // key names of inputs used for calibration
  } ;

  typedef boost::shared_ptr<Base> Handle;

  struct Factory {
    static Handle makeAction(std::string name, const boost::property_tree::ptree& pt) {
      if (name == "FindPeak")
        return Handle(new FindPeak(pt));
      if (name == "Calibrator")
        return Handle(new Calibrator(pt));
      else
        throw std::runtime_error(name+": action not supported");
    }
  } ;
} // namespace Action

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

    virtual void result(std::string resultKey, Result::Base::Handle result) {
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
      fftw_.transformRange(i0, i1, FFT::WindowFunction::Rectangular<double>());
    else if (windowFcnName_ == "Hanning")
      fftw_.transformRange(i0, i1, FFT::WindowFunction::Hanning<double>());
    else if (windowFcnName_ == "Hamming")
      fftw_.transformRange(i0, i1, FFT::WindowFunction::Hamming<double>());
    else if (windowFcnName_ == "Blackman")
      fftw_.transformRange(i0, i1, FFT::WindowFunction::Blackman<double>());
    else 
      throw std::runtime_error(windowFcnName_ + ": unknown window function");      
    
    const Spectrum<FFTFloat> s(fftw_, header.sampleRate(), header.ddcCenterFrequency());
  
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

    for (typename ResultMap::const_iterator i(resultMap.begin()); i!=resultMap.end(); ++i) 
      std::cout << "result: " << i->first << " " << *(i->second) << std::endl;

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

