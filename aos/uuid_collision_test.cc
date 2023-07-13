#include <set>
#include <unordered_set>

#include "glog/logging.h"
#include "gtest/gtest.h"

#include "aos/uuid.h"

namespace aos {
namespace testing {

// Tests that modest numbers of UUID::Random() calls cannot create UUID
// collisions (to test that we have not *completely* messed up the random number
// generation).
TEST(UUIDTest, CollisionTest) {
  std::set<UUID> uuids;
  // When we only had ~32 bits of randomness in our UUIDs, we could generate
  // issues with only ~sqrt(2 ** 32) (aka 2 ** 16) UUIDs.
  // Just go up to 2 ** 22, since too much longer just makes this test take
  // obnoxiously long.
  for (size_t ii = 0; ii < (1UL << 22); ++ii) {
    UUID uuid = UUID::Random();
    ASSERT_FALSE(uuids.count(uuid) > 0) << ii;
    uuids.insert(uuid);
  }
}

// Tests that our random seed generation for the mt19937 does not trivially
// collide.
TEST(UUIDTest, SeedInitializationTest) {
  std::uniform_int_distribution<uint64_t> distribution(0);
  std::set<uint64_t> values;
  // This test takes significantly longer than the above due to needing to query
  // std::random_device substantially. However, covering a range of 2 ** 18
  // should readily catch things if we are accidentally using 32-bit seeds.
  for (size_t ii = 0; ii < (1UL << 18); ++ii) {
    std::mt19937 twister = internal::FullySeededRandomGenerator();
    const uint64_t value = distribution(twister);
    ASSERT_FALSE(values.count(value) > 0) << ii;
    values.insert(value);
  }
}
}  // namespace testing
}  // namespace aos
