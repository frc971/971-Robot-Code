#ifndef FRC971_CONTROL_LOOPS_BINOMIAL_H_
#define FRC971_CONTROL_LOOPS_BINOMIAL_H_

namespace frc971 {
namespace control_loops {

// Computes the factorial of n
constexpr double Factorial(int n) {
  if (n <= 1) {
    return 1.0;
  } else {
    return Factorial(n - 1) * n;
  }
}

// Computes the binomial coefficients.  n choose k.
constexpr double Binomial(int n, int k) {
  return Factorial(n) / (Factorial(k) * Factorial(n - k));
}

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_BINOMIAL_H_
