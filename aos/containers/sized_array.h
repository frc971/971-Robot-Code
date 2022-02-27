#ifndef AOS_CONTAINERS_SIZED_ARRAY_H_
#define AOS_CONTAINERS_SIZED_ARRAY_H_

#include "absl/container/inlined_vector.h"

namespace aos {

// Minimal compliant allocator whose allocating operations are all fatal.
template <typename T>
class FatalAllocator {
 public:
  using value_type = T;

  [[nodiscard, noreturn]] T *allocate(std::size_t) { __builtin_trap(); }

  [[noreturn]] void deallocate(T *, std::size_t) { __builtin_trap(); }
};

// Reuse the logic from absl::InlinedVector for a statically allocated,
// dynamically sized list of values. InlinedVector's default behavior is to
// allocate from the heap when growing beyond the static capacity, make this
// fatal instead to enforce RT guarantees.
template <typename T, size_t N>
using SizedArray = absl::InlinedVector<T, N, FatalAllocator<T>>;

}  // namespace aos

#endif  // AOS_CONTAINERS_SIZED_ARRAY_H_
