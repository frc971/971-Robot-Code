#ifndef AOS_TYPE_TRAITS_
#define AOS_TYPE_TRAITS_

#include <type_traits>

namespace aos {
#if ((__GNUC__ < 5))
namespace {
template<typename Tp>
struct has_trivial_copy_assign : public std::integral_constant<bool,
// This changed between 4.4.5 and 4.6.3. Unless somebody discovers otherwise,
// 4.6 seems like a reasonable place to switch.
#if ((__GNUC__ < 4) || (__GNUC__ == 4 && __GNUC_MINOR__ < 6)) && !defined(__clang__)
    ::std::has_trivial_assign<Tp>::value> {};
#else
    ::std::has_trivial_copy_assign<Tp>::value> {};
#endif

}  // namespace
#endif

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
                    bool,
#if ((__GNUC__ < 5))
                    (::std::has_trivial_copy_constructor<Tp>::value &&
                     ::aos::has_trivial_copy_assign<Tp>::value)
#else
                    (::std::is_trivially_copyable<Tp>::value)
#endif
                    > {
};

}  // namespace aos

#endif
