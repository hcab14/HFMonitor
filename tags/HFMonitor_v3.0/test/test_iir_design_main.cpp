#include <iostream>
#include <iterator>

#include "filter/iir_design.hpp"

void test_iir(size_t N, double fc) {
  filter::iir_design_lowpass iird(N);
  iird.design(N, fc);
  filter::iir_design_lowpass::vector_type a(iird.a());
  filter::iir_design_lowpass::vector_type b(iird.b());
  std::cout << "--------------------------------------------------------------------------------\n";
  std::cout << "a= ";
  std::copy(a.begin(), a.end(), std::ostream_iterator<double>(std::cout, " "));
  std::cout << "\nb= ";
  std::copy(b.begin(), b.end(), std::ostream_iterator<double>(std::cout, " "));
  std::cout << "\n";
}

int main() {
  const double f(0.9);
  for (size_t i=1; i<11; ++i)
    test_iir( i, f);
  std::cout << "--------------------------------------------------------------------------------\n";

#if 0 
  const size_t N(1);
  filter::iir_coefficients iirc(N);
  filter::detail::delta::vector_type alpha;
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
