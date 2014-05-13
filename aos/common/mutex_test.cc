#include "aos/common/mutex.h"

#include <sched.h>
#include <math.h>
#include <pthread.h>

#include "gtest/gtest.h"

#include "aos/linux_code/ipc_lib/aos_sync.h"
#include "aos/common/die.h"
#include "aos/common/util/death_test_log_implementation.h"
#include "aos/common/util/thread.h"
#include "aos/common/time.h"

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

// Sees what happens with multiple unlocks.
TEST_F(MutexDeathTest, RepeatUnlock) {
  test_mutex.Lock();
  test_mutex.Unlock();
  EXPECT_DEATH(
      {
        logging::AddImplementation(new util::DeathTestLogImplementation());
        test_mutex.Unlock();
      },
      ".*multiple unlock.*");
}

// Sees what happens if you unlock without ever locking (or unlocking) it.
TEST_F(MutexDeathTest, NeverLock) {
  EXPECT_DEATH(
      {
        logging::AddImplementation(new util::DeathTestLogImplementation());
        test_mutex.Unlock();
      },
      ".*multiple unlock.*");
}

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

namespace {

class AdderThread : public ::aos::util::Thread {
 public:
  AdderThread(int *counter, Mutex *mutex, ::aos::time::Time sleep_before_time,
              ::aos::time::Time sleep_after_time)
      : counter_(counter),
        mutex_(mutex),
        sleep_before_time_(sleep_before_time),
        sleep_after_time_(sleep_after_time) {}
  virtual void Run() override {
    ::aos::time::SleepFor(sleep_before_time_);
    MutexLocker locker(mutex_);
    ++(*counter_);
    ::aos::time::SleepFor(sleep_after_time_);
  }

 private:
  int *const counter_;
  Mutex *const mutex_;
  const ::aos::time::Time sleep_before_time_, sleep_after_time_;
};

}  // namespace

// Verifies that ThreadSanitizer understands that a contended mutex establishes
// a happens-before relationship.
TEST_F(MutexTest, ThreadSanitizerContended) {
  int counter = 0;
  AdderThread threads[2]{
      {&counter, &test_mutex, ::aos::time::Time::InSeconds(1),
       ::aos::time::Time::InSeconds(0)},
      {&counter, &test_mutex, ::aos::time::Time::InSeconds(0),
       ::aos::time::Time::InSeconds(0)}, };
  for (auto &c : threads) {
    c.Start();
  }
  for (auto &c : threads) {
    c.WaitUntilDone();
  }
  EXPECT_EQ(2, counter);
}

// Verifies that ThreadSanitizer understands that an uncontended mutex
// establishes a happens-before relationship.
TEST_F(MutexTest, ThreadSanitizerUncontended) {
  int counter = 0;
  AdderThread threads[2]{
      {&counter, &test_mutex, ::aos::time::Time::InSeconds(1),
       ::aos::time::Time::InSeconds(0)},
      {&counter, &test_mutex, ::aos::time::Time::InSeconds(0),
       ::aos::time::Time::InSeconds(0)}, };
  for (auto &c : threads) {
    c.Start();
  }
  for (auto &c : threads) {
    c.WaitUntilDone();
  }
  EXPECT_EQ(2, counter);
}

}  // namespace testing
}  // namespace aos
