#include <iostream>
#include <iterator>
#include <vector>
#include <map>
#include <cmath>

#include <complex>

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

class iir_coefficients {
public:
  typedef std::complex<double> complex_type;
  typedef std::vector<double> vector_type;
  iir_coefficients(size_t order)
  : order_(order) {}

  size_t order() const { return order_; }

  double operator()(double s) const {
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

class iir_design {
public:
  typedef std::vector<double> vector_type;

  iir_design(size_t order,
	     double f) // normalized frequency
  : iir_coefficients_(order)
  , delta_(0, make_alpha(order))
  , omega_(tan(M_PI*f)) {}

  int order() const { return iir_coefficients_.order(); }

  vector_type a() const {
    vector_type as;
    const double dx(1/sqrt(17));
    double f(1);
    for (int j=0; j<2*order()+1; ++j) {
      f *= (j>1) ? j : 1;
      f *= (j>0) ? dx : 1;
      double sum(0);
      for (int i=0; i<2*order()+1; ++i) {
	sum += p(dx*delta_.alpha()[i]) * delta_.coeff(j,2*order(), i);
      }
      as.push_back(sum/f);
    }
    return as;
  }
  vector_type b() const {
    vector_type bs;
    const double dx(.9);//1/sqrt(17));
    double f(1);
    for (int j=0; j<2*order()+1; ++j) {
      f *= (j>1) ? j : 1;
      f *= (j>0) ? dx : 1;
      double sum(0);
      for (int i=0; i<2*order()+1; ++i) {
	sum += q(dx*delta_.alpha()[i]) * delta_.coeff(j,2*order(), i);
      }
      bs.push_back(sum/f);
    }
    return bs;
  }

protected:
  // zi = z^{-1}
  // H(zi) = p(zi) / q(zi)
  double p(double zi) const {
    return std::pow((1+zi), order());
  }
  double q(double zi) const {
    return iir_coefficients_((1-zi)/(1+zi)/omega_)*p(zi);
  }

private:
  static delta::vector_type make_alpha(int N) {
    delta::vector_type alpha(2*N+1, 0.);
    for (int i(-N); i<N+1; ++i)
      alpha[i+N] = double(i);
    return alpha;
  }
  iir_coefficients iir_coefficients_;
  delta            delta_;
  double           omega_;
} ;

void test_iir(size_t N, double fc) {
  iir_design iird(N, fc);
  iir_design::vector_type a(iird.a());
  iir_design::vector_type b(iird.b());
  std::cout << "--------------------------------------------------------------------------------\n";
  std::cout << "a= ";
  std::copy(a.begin(), a.end(), std::ostream_iterator<double>(std::cout, " "));
  std::cout << "\nb= ";
  std::copy(b.begin(), b.end(), std::ostream_iterator<double>(std::cout, " "));
  std::cout << "\n";
}

int main() {

  test_iir(1, 0.1);
  test_iir(2, 0.1);
  test_iir(3, 0.1);
  test_iir(4, 0.1);
  test_iir(5, 0.1);
  test_iir(6, 0.1);
  test_iir(7, 0.1);
  test_iir(8, 0.1);
  test_iir(9, 0.1);
  test_iir(10, 0.1);
  std::cout << "--------------------------------------------------------------------------------\n";

#if 0 
  const size_t N(1);
  iir_coefficients iirc(N);
  delta::vector_type alpha;
  for (int i(-N); i<N+1; ++i)
    alpha.push_back(i);
  double x0 = 0;

  delta d(x0, alpha);


  const double dx=1/sqrt(17);
  double f(1);
  for (int j=0; j<2*N+1; ++j) {
    f *= (j>1) ? j : 1;
    f *= (j>0) ? dx : 1;
    double sum(0);
    for (int i=0; i<2*N+1; ++i) {
      sum += iirc(dx*alpha[i]) * d.coeff(j,2*N, i);
    }
    std::cout << j << " : " << sum/f << std::endl;
  }
#endif
  return 1;
}
