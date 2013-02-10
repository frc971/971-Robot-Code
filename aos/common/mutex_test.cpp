#include "aos/common/mutex.h"

#include "gtest/gtest.h"

namespace aos {
namespace testing {

class MutexTest : public ::testing::Test {
 public:
  Mutex test_mutex;
};

typedef MutexTest MutexDeathTest;

TEST_F(MutexTest, TryLock) {
  EXPECT_TRUE(test_mutex.TryLock());
  EXPECT_FALSE(test_mutex.TryLock());
}

TEST_F(MutexTest, Lock) {
  test_mutex.Lock();
  EXPECT_FALSE(test_mutex.TryLock());
}

TEST_F(MutexTest, Unlock) {
  test_mutex.Lock();
  EXPECT_FALSE(test_mutex.TryLock());
  test_mutex.Unlock();
  EXPECT_TRUE(test_mutex.TryLock());
}

#ifndef __VXWORKS__
// Sees what happens with multiple unlocks.
TEST_F(MutexDeathTest, RepeatUnlock) {
  test_mutex.Lock();
  test_mutex.Unlock();
  EXPECT_DEATH(test_mutex.Unlock(), ".*multiple unlock.*");
}

// Sees what happens if you unlock without ever locking (or unlocking) it.
TEST_F(MutexDeathTest, NeverLock) {
  EXPECT_DEATH(test_mutex.Unlock(), ".*multiple unlock.*");
}
#endif

TEST_F(MutexTest, MutexLocker) {
  {
    aos::MutexLocker locker(&test_mutex);
    EXPECT_FALSE(test_mutex.TryLock());
  }
  EXPECT_TRUE(test_mutex.TryLock());
}

}  // namespace testing
}  // namespace aos
