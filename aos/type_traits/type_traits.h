#ifndef AOS_TYPE_TRAITS_
#define AOS_TYPE_TRAITS_

#include <type_traits>

namespace aos {

// A class template that determines whether or not it is safe to pass a type
// through the shared memory system (aka whether or not you can memcpy it).
// Useful in combination with static_assert.
//
// Doesn't need a trivial constructor because it's bytes only need to get
// copied. If it has a non-trivial destructor, somebody has to make sure to call
// it when appropriate.
// See also (3.9) [basic.types] in the C++11 standard.
template <typename Tp>
struct shm_ok : public std::integral_constant<
                    bool, (::std::is_trivially_copy_constructible<Tp>::value)> {
};

}  // namespace aos

#endif
