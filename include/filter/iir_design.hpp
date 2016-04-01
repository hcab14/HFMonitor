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
#ifndef _IIR_DESIGN_HPP_cm131126_
#define _IIR_DESIGN_HPP_cm131126_

#include <complex>
#include <cmath>
#include <map>
#include <utility>
#include <vector>

namespace filter {
  namespace detail {
    class delta {
    public:
      typedef std::vector<double> vector_type;
      
      typedef std::pair<int, std::pair<int, int> > val_type;
      typedef std::map<val_type, double> coeff_map_type;
      typedef std::map<std::pair<int,double>, double> omega_map_type;
      
      delta(double x0,
            const vector_type& alpha)
        : x0_(x0)
        , alpha_(alpha) {}

      const vector_type& alpha() const { return alpha_; }
      
      double coeff(int m, int n, int nu) const {
        const val_type val(std::make_pair(m,std::make_pair(n,nu)));
        coeff_map_type::const_iterator i(coeff_map_.find(val));
        if (i != coeff_map_.end())
          return i->second;
        if (m==0 && n==0 && nu==0)
          return (coeff_map_[val]=
                  1.);
        if (m<0 || nu<0 || n<0)
          return 0.;
        if (nu < n) {
          return (coeff_map_[val]=
                  1/(alpha_[n]-alpha_[nu]) * 
                  ((alpha_[n]-x0_)*coeff(m,n-1,nu) - m*coeff(m-1,n-1,nu)));
        } else { // n ==nu
          if (n==0) return 0;
          return (coeff_map_[val]= 
                  omega(n-2, alpha_[n-1])/omega(n-1, alpha_[n]) *
                  (m*coeff(m-1,n-1,n-1) - (alpha_[n-1]-x0_)*coeff(m,n-1,n-1)));
        }
      }

    protected:
      double omega(int n, double x) const {
        const std::pair<int,double> val(std::make_pair(n,x));
        omega_map_type::const_iterator i(omega_map_.find(val));
        if (i != omega_map_.end())
          return i->second;
        double w(1);
        for (int k(0); k<n+1; ++k)
          w *= (x-alpha_[k]);
        return (omega_map_[val]=
                w);
      }
    private:
      double      x0_;
      vector_type alpha_;
      mutable coeff_map_type coeff_map_;
      mutable omega_map_type omega_map_;
    } ;
  }

  // butterworth poles
  class iir_coefficients {
  public:
    typedef std::complex<double> complex_type;
    typedef std::vector<double> vector_type;
    iir_coefficients(size_t order)
    : order_(order) {}

    size_t order() const { return order_; }

    double eval(double s) const {
#if 1
      double p(1);
      for (size_t n(0); n<order_/2; ++n)
        p *= (s*s + 2*s*std::sin(M_PI*double(int(2*n)+1)/double(2*order_)) + 1.);
      if ((order_ % 2) == 1)
        p *= (s+1);
      return p;
#else
      complex_type p(1,0);
      for (size_t n(0); n<order_; ++n)
        p *= (s - std::exp(M_PI*complex_type(0, double(2*n+1+order_)/double(2*order_))));
      return p.real();
#endif
    }

  protected:
  private:
    size_t order_;
  } ;

  class iir_design_base {
  public:
    typedef std::vector<double> vector_type;
  
    iir_design_base(size_t order) // filter order
    : iir_coefficients_(order)
    , delta_(0, make_alpha(order)) {}

    virtual ~iir_design_base() {}

    size_t order() const { return iir_coefficients_.order(); }

    const vector_type a() const { return a_; }
    const vector_type b() const { return b_; }

  protected:
    // zi = z^{-1}
    // H(zi) = p(zi) / q(zi)
    virtual double p(double zi) const = 0;
    virtual double q(double zi) const = 0;

    bool design(size_t ord) { // filter order
      if (ord != order()) {
        iir_coefficients_ = iir_coefficients(ord);
        delta_            = detail::delta(0, make_alpha(ord));
      }
      a_ = compute_a();
      b_ = compute_b();
      const double norm(a_[0]);
      for (size_t i(0), n(order()+1); i<n; ++i) {
        a_[i] /= norm;
        b_[i] /= norm;
      }
      return true;
    }

    vector_type compute_a() const {
      vector_type as;
      const double dx(1/sqrt(17));
      double f(1);
      for (int j(0), n(order()+1); j<n; ++j) {
        f *= (j>1) ? j : 1;
        f *= (j>0) ? dx : 1;
        double sum(0);
        for (size_t i(0), n(2*order()+1); i<n; ++i)
          sum += q(dx*delta_.alpha()[i]) * delta_.coeff(j,2*order(), i);
        as.push_back(sum/f);
      }
      return as;
    }
    vector_type compute_b() const {
      vector_type bs;
      const double dx(1/sqrt(17));
      double f(1);
      for (int j(0), n(order()+1); j<n; ++j) {
        f *= (j>1) ? j : 1;
        f *= (j>0) ? dx : 1;
        double sum(0);
        for (size_t i(0), n(2*order()+1); i<n; ++i)
          sum += p(dx*delta_.alpha()[i]) * delta_.coeff(j,2*order(), i);
        bs.push_back(sum/f);
      }
      return bs;
    }

    iir_coefficients iir_coefficients_;

  private:
    static detail::delta::vector_type make_alpha(int N) {
      detail::delta::vector_type alpha(2*N+1, 0.);
      for (int i(-N); i<N+1; ++i)
        alpha[i+N] = double(i);
      return alpha;
    }
    detail::delta    delta_;
    vector_type      a_;
    vector_type      b_;
  } ;

  class iir_design_lowpass : public iir_design_base {
  public:
    iir_design_lowpass(size_t order)
    : iir_design_base(order)
    , omega_(0) {}
    virtual ~iir_design_lowpass() {}

    bool design(size_t order, // filter order
                double f) {   // normalized cutoff frequency
      omega_ = std::tan(M_PI*f/2);
      return iir_design_base::design(order);
    }
    
  protected:
    virtual double p(double zi) const {
      return std::pow((1+zi), int(order()));
    }
    virtual double q(double zi) const {
      return iir_coefficients_.eval((1-zi)/(1+zi)/omega_)*p(zi);
    }
  private:
    double omega_;
  } ;
} // namespace filter

#endif // _IIR_DESIGN_HPP_cm131126_
