#ifndef AOS_COMMON_TYPE_TRAITS_
#define AOS_COMMON_TYPE_TRAITS_

#ifndef __VXWORKS__
#include <type_traits>
#else
#include "aos/crio/type_traits/type_traits"
#endif

namespace aos {

// A class template that determines whether or not it is safe to pass a type
// through the shared memory system (aka whether or not you can memcpy it).
// Useful in combination with static_assert.
// Always true when using the cRIO compiler.
//
// Doesn't need a trivial constructor because it's bytes only need to get
// copied, so it does need to not require anything special to be cleaned up
// (trivial destructor).
// See also (3.9) [basic.types] in the C++11 standard.
template<typename Tp>
struct has_trivial_copy_assign : public std::integral_constant<bool,
// This changed between 4.4.5 and 4.6.3. Unless somebody discovers otherwise,
// 4.6 seems like a reasonable place to switch.
#if ((__GNUC__ < 4) || (__GNUC_MINOR__ < 6)) && !defined(__clang__)
    std::has_trivial_assign<Tp>::value> {};
#else
    std::has_trivial_copy_assign<Tp>::value> {};
#endif
template<typename Tp>
struct shm_ok : public std::integral_constant<bool,
    (std::has_trivial_copy_constructor<Tp>::value &&
     aos::has_trivial_copy_assign<Tp>::value &&
     std::has_trivial_destructor<Tp>::value)> {};

}  // namespace aos

#endif
