#include "aos/common/mutex.h"

#include <sched.h>
#include <math.h>
#include <pthread.h>

#include <thread>

#include "gtest/gtest.h"

#include "aos/linux_code/ipc_lib/aos_sync.h"
#include "aos/common/die.h"
#include "aos/common/util/death_test_log_implementation.h"
#include "aos/common/util/thread.h"
#include "aos/common/time.h"
#include "aos/common/queue_testutils.h"
#include "aos/linux_code/ipc_lib/core_lib.h"

namespace aos {
namespace testing {

class MutexTest : public ::testing::Test {
 public:
  Mutex test_mutex;

 protected:
  void SetUp() override {
    ::aos::common::testing::EnableTestLogging();
    SetDieTestMode(true);
  }
};

typedef MutexTest MutexDeathTest;
typedef MutexTest MutexLockerTest;
typedef MutexTest MutexLockerDeathTest;
typedef MutexTest IPCMutexLockerTest;
typedef MutexTest IPCMutexLockerDeathTest;

TEST_F(MutexTest, TryLock) {
  EXPECT_EQ(Mutex::State::kLocked, test_mutex.TryLock());
  EXPECT_EQ(Mutex::State::kUnlocked, test_mutex.TryLock());

  test_mutex.Unlock();
}

TEST_F(MutexTest, Lock) {
  ASSERT_FALSE(test_mutex.Lock());
  EXPECT_EQ(Mutex::State::kUnlocked, test_mutex.TryLock());

  test_mutex.Unlock();
}

TEST_F(MutexTest, Unlock) {
  ASSERT_FALSE(test_mutex.Lock());
  EXPECT_EQ(Mutex::State::kUnlocked, test_mutex.TryLock());
  test_mutex.Unlock();
  EXPECT_EQ(Mutex::State::kLocked, test_mutex.TryLock());

  test_mutex.Unlock();
}

// Sees what happens with multiple unlocks.
TEST_F(MutexDeathTest, RepeatUnlock) {
  logging::Init();
  ASSERT_FALSE(test_mutex.Lock());
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
  logging::Init();
  EXPECT_DEATH(
      {
        logging::AddImplementation(new util::DeathTestLogImplementation());
        test_mutex.Unlock();
      },
      ".*multiple unlock.*");
}

// Sees what happens with multiple locks.
TEST_F(MutexDeathTest, RepeatLock) {
  EXPECT_DEATH(
      {
        logging::AddImplementation(new util::DeathTestLogImplementation());
        ASSERT_FALSE(test_mutex.Lock());
        ASSERT_FALSE(test_mutex.Lock());
      },
      ".*multiple lock.*");
}

TEST_F(MutexDeathTest, DestroyLocked) {
  EXPECT_DEATH(
      {
        logging::AddImplementation(new util::DeathTestLogImplementation());
        Mutex new_mutex;
        ASSERT_FALSE(new_mutex.Lock());
      },
      ".*destroying locked mutex.*");
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
      {&counter, &test_mutex, ::aos::time::Time::InSeconds(0.2),
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

// Verifiers that ThreadSanitizer understands how a mutex works.
// For some reason this used to fail when the other tests didn't...
TEST_F(MutexTest, ThreadSanitizerMutexLocker) {
  int counter = 0;
  ::std::thread thread([&counter, this]() {
    for (int i = 0; i < 1000; ++i) {
      MutexLocker locker(&test_mutex);
      ++counter;
    }
  });
  for (int i = 0; i < 1000; ++i) {
    MutexLocker locker(&test_mutex);
    --counter;
  }
  thread.join();
  EXPECT_EQ(0, counter);
}

// Verifies that ThreadSanitizer understands that an uncontended mutex
// establishes a happens-before relationship.
TEST_F(MutexTest, ThreadSanitizerUncontended) {
  int counter = 0;
  AdderThread threads[2]{
      {&counter, &test_mutex, ::aos::time::Time::InSeconds(0.2),
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

TEST_F(MutexLockerTest, Basic) {
  {
    aos::MutexLocker locker(&test_mutex);
    EXPECT_EQ(Mutex::State::kUnlocked, test_mutex.TryLock());
  }
  EXPECT_EQ(Mutex::State::kLocked, test_mutex.TryLock());

  test_mutex.Unlock();
}

TEST_F(IPCMutexLockerTest, Basic) {
  {
    aos::IPCMutexLocker locker(&test_mutex);
    EXPECT_EQ(Mutex::State::kUnlocked, test_mutex.TryLock());
    EXPECT_FALSE(locker.owner_died());
  }
  EXPECT_EQ(Mutex::State::kLocked, test_mutex.TryLock());

  test_mutex.Unlock();
}

TEST_F(IPCMutexLockerDeathTest, NoCheckOwnerDied) {
  EXPECT_DEATH({ aos::IPCMutexLocker locker(&test_mutex); },
               "nobody checked if the previous owner of mutex [^ ]+ died.*");
}

}  // namespace testing
}  // namespace aos
