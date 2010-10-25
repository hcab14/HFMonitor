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

class SpectrumBase {
public:
  typedef std::complex<double> Complex;

  SpectrumBase(double sampleRate,
               double centerFrequency)
    : sampleRate_(sampleRate)
    , centerFrequency_(centerFrequency) {}
  virtual ~SpectrumBase() {}

  double sampleRate() const { return sampleRate_; }
  double centerFrequency() const { return centerFrequency_; }

  virtual size_t size()                    const = 0;
  virtual Complex operator[](size_t index) const = 0;
  
  size_t freq2Index(double qrg_Hz) const { // get the nearest bin index
    const int n(size());
    const int i(round(double(n)*(qrg_Hz - centerFrequency()) / sampleRate()));
    if (i >= -n/2 && i < n/2)
      return (i>=0) ? i : n+i;
    else 
      throw std::runtime_error("freq2Index failed");
  }
  
  double index2Freq(size_t index) const {
    const int n(size());
    return centerFrequency() + sampleRate() * (int(index)>=n/2 ? -n+int(index) : int(index)) / double(n);
  }  
private:
  static int round(double d) { return int(d+((d>=0.0) ? .5 : -.5)); }

  double sampleRate_;
  double centerFrequency_;
} ;

template<typename T>
class FFTWSpectrum : public SpectrumBase {
public:
  FFTWSpectrum(const FFT::FFTWTransform<T>& fftw,
               double sampleRate,
               double centerFrequency)
    : SpectrumBase(sampleRate, centerFrequency)
    , fftw_(fftw) {}
  virtual ~FFTWSpectrum() {}

  virtual size_t size() const { return fftw_.size(); }
  virtual std::complex<double> operator[](size_t index) const { return fftw_.getBin(index); }  
protected:
private:
  const FFT::FFTWTransform<T>& fftw_;
  double sampleRate_;
  double centerFrequency_;
} ;

// class FreqStrength {
// public:
//   FreqStrength(double freq, double strength)
//     : freq_(freq)
//     , strength_(strength) {}
  
//   double freq() const { return freq_; }
//   double strength() const { return strength_; }
//   double& strength() { return strength_; }
  
//   static bool cmpStrength(const FreqStrength& fs1, 
//                           const FreqStrength& fs2) {
//     return fs1.strength() < fs2.strength();
//   }
//   static bool cmpFreq(const FreqStrength& fs1, 
//                       const FreqStrength& fs2) {
//     return fs1.freq() < fs2.freq();
//   }
// private:
//   double freq_;
//   double strength_;
// } ;

class PowerSpectrum {
public:
  typedef std::pair<double, double> FreqStrength;
  typedef std::vector<FreqStrength> Vec;
  typedef Vec::iterator       iterator;
  typedef Vec::const_iterator const_iterator;

  PowerSpectrum(double fMin, double fMax)
    : fMin_(fMin)
    , fMax_(fMax) {}

  PowerSpectrum(double fMin, double fMax, const SpectrumBase& s)
    : fMin_(fMin)
    , fMax_(fMax) {
    fill(s);
  }
  
  double fMin() const { return fMin_; }
  double fMax() const { return fMax_; }

  void fill(const SpectrumBase& s) {
    ps_.clear();
    const size_t i0(s.freq2Index(fMin_));
    const size_t i1(s.freq2Index(fMax_));
    for (size_t u=i0; u<=i1; ++u) 
      ps_.push_back(FreqStrength(s.index2Freq(u), std::abs(s[u])));
  }
  const FreqStrength& operator[](unsigned index) const { return ps_[index]; }
  void clear() { ps_.clear(); }
  size_t size() const { return ps_.size(); }
  bool empty() const { return ps_.empty(); }
  iterator       begin()       { return ps_.begin(); }
  const_iterator begin() const { return ps_.begin(); }
  iterator       end()       { return ps_.end(); }
  const_iterator end() const { return ps_.end(); }

  static bool cmpFreq(const FreqStrength& fs1,
                      const FreqStrength& fs2) {
    return fs1.first < fs2.first;
  }
  static bool cmpStrength(const FreqStrength& fs1,
                          const FreqStrength& fs2) {
    return fs1.second < fs2.second;
  }

  PowerSpectrum& operator+=(const PowerSpectrum& ps) {
    if (size() != ps.size())
      throw 1; // TODO
    const_iterator j(ps.begin());
    for (iterator i(begin()); i!=end(); ++i,++j) {
      if (i->first != j->first) throw 1; // TODO
      i->second += j->second;
    }
    return *this;
  }
  PowerSpectrum& operator-=(const PowerSpectrum& ps) {
    if (size() != ps.size())
      throw 1; // TODO
    const_iterator j(ps.begin());
    for (iterator i(begin()); i!=end(); ++i,++j) {
      if (i->first != j->first) throw 1; //TODO
      i->second -= j->second;
    }
    return *this;
  }
  PowerSpectrum& operator*=(double f) {
    for (iterator i(begin()); i!=end(); ++i)
      i->second *= f;
    return *this;
  }
  friend PowerSpectrum operator*(const PowerSpectrum& p,
                                 double f) {
    PowerSpectrum r(p); r*=f;
    return r;
  }
  friend PowerSpectrum operator*(double f,
                                 const PowerSpectrum& p) {    
    PowerSpectrum r(p); r*=f;
    return r;
  }
private:
  const double fMin_;
  const double fMax_;
  Vec ps_;
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

    friend std::ostream& operator<<(std::ostream& os, const Base& b) {
      return os << b.toString();
    }

    void dump(std::string path,
              std::string tag,
              boost::posix_time::ptime t) const {      
      boost::filesystem::path p(getFilePath(path, tag, t));
      const bool writeHeader(not boost::filesystem::exists(p));
      boost::filesystem::ofstream ofs(p, std::ios::app);
      if (writeHeader)
        dumpHeader(ofs, t) << lineBreak();
      dumpData(ofs, t) << lineBreak();
    }
  protected:
    virtual std::string lineBreak() const { return "\n"; }
    virtual std::ostream& dumpData(std::ostream& os,
                                   boost::posix_time::ptime t) const {
      return os << boost::posix_time::to_iso_string(t) << " ";
    }
    virtual std::ostream& dumpHeader(std::ostream& os,
                                     boost::posix_time::ptime t) const {
      return os << "# Time ";
    }
    virtual boost::filesystem::path getFilePath(std::string basePath,
                                                std::string tag,
                                                boost::posix_time::ptime t) const {
      using namespace boost::gregorian;
      boost::filesystem::path p(basePath+"/"+tag);
      boost::filesystem::create_directories(p);
      std::stringstream oss;
      oss.imbue(std::locale(oss.getloc(), new date_facet("%Y-%m-%d")));
      oss << "/" << t.date() << "." << fileExtenstion();
      return p/=(oss.str());
    }
    virtual std::string fileExtenstion() const {
      return "txt";
    }
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

    bool findPeak(const PowerSpectrum& ps, double minRatio) {
      PowerSpectrum::const_iterator
        iMin(std::min_element(ps.begin(), ps.end(), PowerSpectrum::cmpStrength));
      PowerSpectrum::const_iterator
        iMax(std::max_element(ps.begin(), ps.end(), PowerSpectrum::cmpStrength));
      
      const size_t indexMax(std::distance(ps.begin(), iMax));

      double sum(0.0);
      double weight(0.0);
      for (size_t u(indexMax-std::min(indexMax, size_t(3))); u<(indexMax+4) && u < ps.size(); ++u) {
        weight += ps[u].second;
        sum += ps[u].second * ps[u].first;
      }
      ratio_ = (iMin->second != 0.0) ? iMax->second / iMin->second : 1.0;
      if (ratio_ < minRatio || weight == 0.0) {
        std::cout << "ratio < minRatio || weight == 0.0: " << ratio_ << " " << sum << " " << weight << std::endl;
        return false;
      }
      // TODO: error propagation
      const std::pair<double, double> c(cal(sum/weight));
      fMeasured_    = c.first;
      fMeasuredRMS_ = c.second;
      strength_     = iMax->second;
      strengthRMS_  = iMax->second/10.; // TODO
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

    virtual std::ostream& dumpData(std::ostream& os,
                                   boost::posix_time::ptime t) const {
      return Base::dumpData(os, t)
        << boost::format("%12.3f") % fReference() << " "
        << boost::format("%12.3f") % fMeasured() << " "
        << boost::format("%6.3f")  % fMeasuredRMS() << " "
        << boost::format("%7.2f")  % (20.*std::log10(strength())) << " "
        << boost::format("%7.2f")  % (20.*std::log10(strengthRMS())) << " "
        << boost::format("%5.2f")  % std::log10(ratio()) << " ";
    }
    virtual std::ostream& dumpHeader(std::ostream& os,
                                     boost::posix_time::ptime t) const {      
      return Base::dumpHeader(os, t) 
        << "fReference_Hz fMeasured_Hz fMeasuredRMS_Hz strength_dB strengthRMS_dB log10(ratio) ";
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
    
    virtual std::ostream& dumpData(std::ostream& os,
                                   boost::posix_time::ptime t) const {
      return Base::dumpData(os, t) 
        << boost::format("%6.3f")  % x_(0) << " "
        << boost::format("%10.2f") % (1e6*(1-x_(1))) << " "
        << boost::format("%7.3f")  % std::sqrt(q_(0,0)) << " "
        << boost::format("%7.2f")  % (1e6*std::sqrt(q_(1,1))) << " ";
    }
    virtual std::ostream& dumpHeader(std::ostream& os,
                                     boost::posix_time::ptime t) const {      
      return Base::dumpHeader(os, t) 
        << " clockOffset_Hz clockOffset_ppm clockOffsetRMS_Hz clockOffsetRMS_ppm";
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
         << " diff=" << fMeasured()-fReference();
      return ss.str();
    }

    virtual std::pair<double, double> cal(double f) const {
      return calibrationHandle_->uncal2cal(f);
    }

    virtual std::ostream& dumpData(std::ostream& os,
                                   boost::posix_time::ptime t) const {
      return SpectrumPeak::dumpData(os, t) 
        << boost::format("%8.3f")  % (fMeasured()-fReference()) << " ";
    }
    virtual std::ostream& dumpHeader(std::ostream& os,
                                     boost::posix_time::ptime t) const {      
      return SpectrumPeak::dumpHeader(os, t) 
        << "diff_Hz ";
    }

  private:
    Calibration::Handle calibrationHandle_;
  } ;
} // namespace Result

namespace Proxy {
  class Base {
  public:
    virtual ~Base() {}
    virtual void putResult(std::string resultKey, Result::Base::Handle result) {
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
      , ps_(config.get<double>("fMin"), 
            config.get<double>("fMax"))
      , filterType_(config.find("Filter") != config.not_found() 
                    ? config.get<std::string>("Filter.Type")
                    : "None")
      , filterTimeConstant_((filterType_ != "None") 
                            ? config.get<double>("Filter.TimeConstant")
                            : 1.0) {}
    
    virtual void perform(Proxy::Base& p, const SpectrumBase& s) {
      try {
        if (filterType_ == "None") {
          ps_.clear();
          ps_.fill(s);
        } else if (filterType_ == "LowPass") {
          if (ps_.empty()) {
            ps_.fill(s);
        } else {
            const double dt(s.size()/s.sampleRate());
            const double x(dt / filterTimeConstant_);
            ps_ *= (1-x);
            ps_ += x * PowerSpectrum(ps_.fMin(), ps_.fMax(), s);
        }
      } else {
        throw std::runtime_error(filterType_ + " unknown filter");
      }
      // call virtual method
      proc(p, s, ps_);      
      } catch (...) {
        return;
      }
    }

    // this method is overwritten by, e.g., FindPeak, see below
    virtual void proc(Proxy::Base& p, 
                      const SpectrumBase& s,
                      const PowerSpectrum& ps) = 0;
    
  protected:
  private:
    PowerSpectrum ps_;
    const std::string filterType_;
    const double filterTimeConstant_;
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
                      const PowerSpectrum& ps) {
      std::cout << "FindPeak::perform " << std::endl;
      try {
        Result::SpectrumPeak::Handle 
          spp((useCalibration_)
              ? boost::make_shared<
                Result::CalibratedSpectrumPeak>(fReference_,
                                                boost::dynamic_pointer_cast<Result::Calibration>
                                                (p.getResult(calibrationKey_)))
              : boost::make_shared<Result::SpectrumPeak>(fReference_));
        if (spp->findPeak(ps, minRatio_)) {
          p.putResult(resultKey_, spp);
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
        p.putResult(resultKey_, rh); 
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

