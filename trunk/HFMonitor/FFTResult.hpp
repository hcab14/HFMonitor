// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_RESULT_HPP_cm101026_
#define _FFT_RESULT_HPP_cm101026_

#include <string>
#include <boost/filesystem.hpp>

#include "Spectrum.hpp"

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

#endif // _FFT_RESULT_HPP_cm101026_