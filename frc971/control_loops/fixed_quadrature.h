#ifndef FRC971_CONTROL_LOOPS_FIXED_QUADRATURE_H_
#define FRC971_CONTROL_LOOPS_FIXED_QUADRATURE_H_

#include <array>

namespace frc971 {
namespace control_loops {

// Implements Gaussian Quadrature integration (5th order).  fn is the function to
// integrate.  It must take 1 argument of type T.  The integration is between a
// and b.
template <typename F, typename T>
T GaussianQuadrature5(const F &fn, T a, T b) {
  // Pulled from Python.
  // numpy.set_printoptions(precision=20)
  // scipy.special.p_roots(5)
  const ::std::array<double, 5> x{{
      -9.06179845938663630633e-01, -5.38469310105682885670e-01,
      3.24607628916367383789e-17, 5.38469310105683218737e-01,
      9.06179845938663408589e-01}};

  const ::std::array<double, 5> w{{
      0.23692688505618844652, 0.4786286704993669705, 0.56888888888888811124,
      0.47862867049936674846, 0.23692688505618875183}};

  double answer = 0.0;
  for (int i = 0; i < 5; ++i) {
    const double y = (b - a) * (x[i] + 1) / 2.0 + a;
    answer += (b - a) / 2.0 * w[i] * fn(y);
  }
  return answer;
}

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_FIXED_QUADRATURE_H_
