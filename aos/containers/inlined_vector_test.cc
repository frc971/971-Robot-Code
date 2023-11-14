#include "aos/containers/inlined_vector.h"

#include "gtest/gtest.h"

#include "aos/realtime.h"

DECLARE_bool(die_on_malloc);

namespace aos {
namespace testing {

// Checks that we don't malloc until/unless we need to increase the size of the
// vector.
TEST(SizedArrayTest, NoUnnecessaryMalloc) {
  gflags::FlagSaver flag_saver;
  FLAGS_die_on_malloc = true;
  RegisterMallocHook();
  InlinedVector<int, 5> a;
  {
    aos::ScopedRealtime realtime;
    a.push_back(9);
    a.push_back(7);
    a.push_back(1);
    a.push_back(2);
    a.push_back(3);

    // And double-check that we can actually construct a new object at realtime.
    InlinedVector<int, 5> b;
  }
// Malloc hooks don't work with asan/msan.
#if !__has_feature(address_sanitizer) && !__has_feature(memory_sanitizer)
  EXPECT_DEATH(
      {
        aos::ScopedRealtime realtime;
        a.push_back(4);
      },
      "Malloced");
#endif
}

// Tests that we can create/define a vector with zero statically allocated
// elements (the absl::InlinedVector does not allow this for some reason).
TEST(SizedArrayTest, ZeroLengthVector) {
  InlinedVector<int, 0> zero;
  zero.push_back(1);
  ASSERT_EQ(1, zero[0]);
}
}  // namespace testing
}  // namespace aos
