#include "aos/common/mutex.h"

#include <sched.h>
#include <math.h>
#include <pthread.h>
#ifdef __VXWORKS__
#include <taskLib.h>
#endif

#include "gtest/gtest.h"

#include "aos/atom_code/ipc_lib/aos_sync.h"
#include "aos/common/die.h"

namespace aos {
namespace testing {

class MutexTest : public ::testing::Test {
 public:
  Mutex test_mutex;

 protected:
  void SetUp() override {
    SetDieTestMode(true);
  }
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

TEST_F(MutexTest, MutexUnlocker) {
  test_mutex.Lock();
  {
    aos::MutexUnlocker unlocker(&test_mutex);
    // If this fails, then something weird is going on and the next line might
    // hang, so fail immediately.
    ASSERT_TRUE(test_mutex.TryLock());
    test_mutex.Unlock();
  }
  EXPECT_FALSE(test_mutex.TryLock());
}

}  // namespace testing
}  // namespace aos
