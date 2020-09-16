#ifndef AOS_EVENTS_LOGGING_EIGEN_MPQ_H_
#define AOS_EVENTS_LOGGING_EIGEN_MPQ_H_

#include "Eigen/Dense"
#include "third_party/gmp/gmpxx.h"

namespace Eigen {

// TypeTraits for mpq_class.  This is only really enough to use inverse().
template <>
struct NumTraits<mpq_class>
    : GenericNumTraits<mpq_class> {
  typedef mpq_class Real;
  typedef mpq_class Literal;
  typedef mpq_class NonInteger;
  typedef mpq_class Nested;

  enum {
    IsComplex = 0,
    IsInteger = 0,
    IsSigned = 1,
    RequireInitialization = 1,
    ReadCost = 1,
    AddCost = 3,
    MulCost = 9
  };

  static inline Real dummy_precision() { return mpq_class(0); }
  static inline Real epsilon() { return mpq_class(0); }
  static inline int digits10() { return 0; }
};

}  // namespace Eigen

#endif  // AOS_EVENTS_LOGGING_EIGEN_MPQ_H_
