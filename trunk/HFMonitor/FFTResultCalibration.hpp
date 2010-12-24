// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_RESULT_CALIBRATION_HPP_cm101026_
#define _FFT_RESULT_CALIBRATION_HPP_cm101026_

#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "FFTResultSpectrumPeak.hpp"
#include "logging.hpp"
#include "InvertMatrix.hpp"

namespace Result {
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
      ASSERT_THROW(ublas_util::InvertMatrix(ata, q_) == true);
      x_ = prod(q_, Vector(prod(trans(a),y)));
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
    
    virtual boost::filesystem::fstream& dumpHeader(boost::filesystem::fstream& os,
                                                   boost::posix_time::ptime t) const {      
      Base::dumpHeader(os, t)
        << " clockOffset_Hz clockOffset_ppm clockOffsetRMS_Hz clockOffsetRMS_ppm";
      return os;
    }
    virtual boost::filesystem::fstream& dumpData(boost::filesystem::fstream& os,
                                                 boost::posix_time::ptime t) const {
      Base::dumpData(os, t) 
        << boost::format("%6.3f")  % x_(0) << " "
        << boost::format("%10.2f") % (1e6*(1-x_(1))) << " "
        << boost::format("%7.3f")  % std::sqrt(q_(0,0)) << " "
        << boost::format("%7.2f")  % (1e6*std::sqrt(q_(1,1))) << " ";
      return os;
    }
  private:
    size_t n_;
    Matrix q_;
    Vector x_;
  } ;
} // namespace Result

#endif // _FFT_RESULT_CALIBRATION_HPP_cm101026_
