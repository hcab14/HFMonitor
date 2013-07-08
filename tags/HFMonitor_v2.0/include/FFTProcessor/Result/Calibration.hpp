// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2013 Christoph Mayer
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
#ifndef _FFT_RESULT_CALIBRATION_HPP_cm101026_
#define _FFT_RESULT_CALIBRATION_HPP_cm101026_

#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "logging.hpp"
#include "InvertMatrix.hpp"
#include "FFTProcessor/Result/SpectrumPeak.hpp"

namespace Result {
  class Calibration : public Base {
  public:
    typedef boost::shared_ptr<Calibration> Handle;
    typedef boost::numeric::ublas::matrix<double> Matrix;
    typedef boost::numeric::ublas::vector<double> Vector;

    Calibration(ptime time, 
                const std::vector<Result::SpectrumPeak::Handle>& peaks,
                double offsetMax, 
                double ppmMax,
                size_t n=2)
      : Base("Calibration", time)
      , n_(n)
      , q_(n,n)
      , x_(n)
      , offsetMax_(offsetMax)
      , ppmMax_(ppmMax) {
      using namespace boost::numeric::ublas;
      // add pseudo-observation (0,0) when single measurement
      const bool single_measurement(peaks.size() == 1);
      size_t n_meas(peaks.size() + single_measurement);
      Matrix a(n_meas, n);
      Vector y(n_meas);
      for (size_t u(0); u<peaks.size(); ++u) {
        y(u)   = peaks[u]->fMeasured();
        double t(1.0);
        for (size_t v(0); v<n; ++v) {
          a(u,v) = t; t *= peaks[u]->fReference();
        }
      }
      if (single_measurement) {
        y(1)   = 0;
        a(1,0) = 1;
      }
      // least squares inversion
      Matrix ata(prod(trans(a),a));
      ASSERT_THROW(ublas_util::InvertMatrix(ata, q_) == true);
      x_ = prod(q_, Vector(prod(trans(a),y)));
      ASSERT_THROW(std::abs(x_(0))   < offsetMax_);
      ASSERT_THROW(std::abs(x_(1)-1) < 1e-6*ppmMax_);
    }
    Calibration(ptime time,
                const Matrix& q,
                const Vector& x,
                double offsetMax,
                double ppmMax)
      : Base("Calibration", time)
      , n_(2)
      , q_(q)
      , x_(x)
      , offsetMax_(offsetMax)
      , ppmMax_(ppmMax) {}

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

    // return a handle with a "worst-case" covariance matrix
    Base::Handle withWorstCaseCov() const {
      Matrix q(2,2);
      q(1,0) = q(0,1) = 0.0;
      q(0,0) = std::pow(  offsetMax_, 2);
      q(1,1) = std::pow(1e-6*ppmMax_, 2);
      return Base::Handle(new Calibration(time_, q, x_, offsetMax_, ppmMax_));
    }

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
    
    virtual std::ostream& dumpHeader(std::ostream& os) const {      
      Base::dumpHeader(os);
      return os << " clockOffset_Hz clockOffset_ppm clockOffsetRMS_Hz clockOffsetRMS_ppm";
    }
    virtual std::ostream& dumpData(std::ostream& os) const {
      return os << boost::format("%6.3f")  % x_(0) << " "
                << boost::format("%10.2f") % (1e6*(1-x_(1))) << " "
                << boost::format("%7.3f")  % std::sqrt(q_(0,0)) << " "
                << boost::format("%7.2f")  % (1e6*std::sqrt(q_(1,1))) << " ";
    }
  private:
    size_t n_;
    Matrix q_;
    Vector x_;
    double offsetMax_;
    double ppmMax_;
  } ;
} // namespace Result

#endif // _FFT_RESULT_CALIBRATION_HPP_cm101026_
