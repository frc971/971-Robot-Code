#ifndef AOS_CONTAINERS_INLINED_VECTOR_H_
#define AOS_CONTAINERS_INLINED_VECTOR_H_

#include <stddef.h>

#include <vector>

#include "absl/container/inlined_vector.h"

namespace aos {

template <typename T, size_t N>
struct InlinedVector : public absl::InlinedVector<T, N> {
 public:
  using absl::InlinedVector<T, N>::InlinedVector;
};

// Specialized for the N == 0 case because absl::InlinedVector doesn't support
// it for some reason.
template <typename T>
struct InlinedVector<T, 0> : public std::vector<T> {
 public:
  using std::vector<T>::vector;
};
}  // namespace aos
#endif  // AOS_CONTAINERS_INLINED_VECTOR_H_
