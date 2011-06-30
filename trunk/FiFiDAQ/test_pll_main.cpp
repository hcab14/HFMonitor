#include <iostream>
#include "pll.hpp"

int main()
{
  const double fs(1000.);
  const double fc( 100.);
  const double dwl(  1.);
  const double xi(1./sqrt(2.));
  typedef pll<double> pll_type;
  typedef pll_type::complex_type complex_type;
  pll_type p;
  p.init(xi, dwl, fc, fs);
  // generate input and invoke pll
  const size_t n(10*1000);

  std::cout << "out=[\n";
  for (size_t i=0; i<n; ++i) {
    const double t(double(i)*2*M_PI*fc/fs);
    const complex_type s(std::exp(complex_type(0., t)) 
			 + 0.5*complex_type(drand48()-0.5, drand48()-0.5));

    p.process(s);
    std::cout << s.real() << " " << s.imag() << " " 
	      << p.theta() << " " << p.f1() << " " 
	      << p.uf() << " " << p.ud() << std::endl;
  }
  std::cout << "];\n";
}
