#include "aos/mutex/mutex.h"

#include <math.h>
#include <pthread.h>
#include <sched.h>

#include <chrono>
#include <thread>

#include "gtest/gtest.h"

#include "aos/die.h"
#include "aos/ipc_lib/aos_sync.h"
#include "aos/ipc_lib/core_lib.h"
#include "aos/testing/test_logging.h"
#include "aos/testing/test_shm.h"
#include "aos/time/time.h"
#include "aos/util/death_test_log_implementation.h"

namespace aos {
namespace testing {

namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;

class MutexTest : public ::testing::Test {
 public:
  Mutex test_mutex_;

 protected:
  void SetUp() override {
    ::aos::testing::EnableTestLogging();
    SetDieTestMode(true);
  }
};

typedef MutexTest MutexDeathTest;
typedef MutexTest MutexLockerTest;
typedef MutexTest MutexLockerDeathTest;
typedef MutexTest IPCMutexLockerTest;
typedef MutexTest IPCMutexLockerDeathTest;
typedef MutexTest IPCRecursiveMutexLockerTest;

TEST_F(MutexTest, TryLock) {
  EXPECT_EQ(Mutex::State::kLocked, test_mutex_.TryLock());
  EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());

  test_mutex_.Unlock();
}

TEST_F(MutexTest, Lock) {
  ASSERT_FALSE(test_mutex_.Lock());
  EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());

  test_mutex_.Unlock();
}

TEST_F(MutexTest, Unlock) {
  ASSERT_FALSE(test_mutex_.Lock());
  EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());
  test_mutex_.Unlock();
  EXPECT_EQ(Mutex::State::kLocked, test_mutex_.TryLock());

  test_mutex_.Unlock();
}

// Sees what happens with multiple unlocks.
TEST_F(MutexDeathTest, RepeatUnlock) {
  ASSERT_FALSE(test_mutex_.Lock());
  test_mutex_.Unlock();
  EXPECT_DEATH(
      {
        logging::SetImplementation(
            std::make_shared<util::DeathTestLogImplementation>());
        test_mutex_.Unlock();
      },
      ".*multiple unlock.*");
}

// Sees what happens if you unlock without ever locking (or unlocking) it.
TEST_F(MutexDeathTest, NeverLock) {
  EXPECT_DEATH(
      {
        logging::SetImplementation(
            std::make_shared<util::DeathTestLogImplementation>());
        test_mutex_.Unlock();
      },
      ".*multiple unlock.*");
}

// Tests that locking a mutex multiple times from the same thread fails nicely.
TEST_F(MutexDeathTest, RepeatLock) {
  EXPECT_DEATH(
      {
        logging::SetImplementation(
            std::make_shared<util::DeathTestLogImplementation>());
        ASSERT_FALSE(test_mutex_.Lock());
        ASSERT_FALSE(test_mutex_.Lock());
      },
      ".*multiple lock.*");
}

// Tests that Lock behaves correctly when the previous owner exits with the lock
// held (which is the same as dying any other way).
TEST_F(MutexTest, OwnerDiedDeathLock) {
  testing::TestSharedMemory my_shm;
  Mutex *mutex =
      static_cast<Mutex *>(shm_malloc_aligned(sizeof(Mutex), alignof(Mutex)));
  new (mutex) Mutex();

  std::thread thread([&]() { ASSERT_FALSE(mutex->Lock()); });
  thread.join();
  EXPECT_TRUE(mutex->Lock());

  mutex->Unlock();
  mutex->~Mutex();
}

// Tests that TryLock behaves correctly when the previous owner dies.
TEST_F(MutexTest, OwnerDiedDeathTryLock) {
  testing::TestSharedMemory my_shm;
  Mutex *mutex =
      static_cast<Mutex *>(shm_malloc_aligned(sizeof(Mutex), alignof(Mutex)));
  new (mutex) Mutex();

  std::thread thread([&]() { ASSERT_FALSE(mutex->Lock()); });
  thread.join();
  EXPECT_EQ(Mutex::State::kOwnerDied, mutex->TryLock());

  mutex->Unlock();
  mutex->~Mutex();
}

// TODO(brians): Test owner dying by being SIGKILLed and SIGTERMed.

// This sequence of mutex operations used to mess up the robust list and cause
// one of the mutexes to not get owner-died like it should.
TEST_F(MutexTest, DontCorruptRobustList) {
  // I think this was the allocator lock in the original failure.
  Mutex mutex1;
  // This one should get owner-died afterwards (iff the kernel accepts the
  // robust list and uses it). I think it was the task_death_notification lock
  // in the original failure.
  Mutex mutex2;

  std::thread thread([&]() {
    ASSERT_FALSE(mutex1.Lock());
    ASSERT_FALSE(mutex2.Lock());
    mutex1.Unlock();
  });
  thread.join();

  EXPECT_EQ(Mutex::State::kLocked, mutex1.TryLock());
  EXPECT_EQ(Mutex::State::kOwnerDied, mutex2.TryLock());

  mutex1.Unlock();
  mutex2.Unlock();
}

// Verifies that ThreadSanitizer understands that a contended mutex establishes
// a happens-before relationship.
TEST_F(MutexTest, ThreadSanitizerContended) {
  int counter = 0;
  std::thread thread1([this, &counter]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    MutexLocker locker(&test_mutex_);
    ++counter;
  });
  std::thread thread2([this, &counter]() {
    MutexLocker locker(&test_mutex_);
    ++counter;
  });
  thread1.join();
  thread2.join();
  EXPECT_EQ(2, counter);
}

// Verifiers that ThreadSanitizer understands how a mutex works.
// For some reason this used to fail when the other tests didn't...
// The loops make it fail more reliably when it's going to.
TEST_F(MutexTest, ThreadSanitizerMutexLocker) {
  for (int i = 0; i < 100; ++i) {
    int counter = 0;
    ::std::thread thread([&counter, this]() {
      for (int i = 0; i < 300; ++i) {
        MutexLocker locker(&test_mutex_);
        ++counter;
      }
    });
    for (int i = 0; i < 300; ++i) {
      MutexLocker locker(&test_mutex_);
      --counter;
    }
    thread.join();
    EXPECT_EQ(0, counter);
  }
}

// Verifies that ThreadSanitizer understands that an uncontended mutex
// establishes a happens-before relationship.
TEST_F(MutexTest, ThreadSanitizerUncontended) {
  int counter = 0;
  std::thread thread1([this, &counter]() {
    MutexLocker locker(&test_mutex_);
    ++counter;
  });
  std::thread thread2([this, &counter]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    MutexLocker locker(&test_mutex_);
    ++counter;
  });
  thread1.join();
  thread2.join();
  EXPECT_EQ(2, counter);
}

// Makes sure that we don't SIGSEGV or something with multiple threads.
TEST_F(MutexTest, MultiThreadedLock) {
  std::thread thread([this] {
    ASSERT_FALSE(test_mutex_.Lock());
    test_mutex_.Unlock();
  });
  ASSERT_FALSE(test_mutex_.Lock());
  test_mutex_.Unlock();
  thread.join();
}

TEST_F(MutexLockerTest, Basic) {
  {
    aos::MutexLocker locker(&test_mutex_);
    EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());
  }
  EXPECT_EQ(Mutex::State::kLocked, test_mutex_.TryLock());

  test_mutex_.Unlock();
}

// Tests that MutexLocker behaves correctly when the previous owner dies.
TEST_F(MutexLockerDeathTest, OwnerDied) {
  testing::TestSharedMemory my_shm;
  Mutex *mutex =
      static_cast<Mutex *>(shm_malloc_aligned(sizeof(Mutex), alignof(Mutex)));
  new (mutex) Mutex();

  std::thread thread([&]() { ASSERT_FALSE(mutex->Lock()); });
  thread.join();
  EXPECT_DEATH(
      {
        logging::SetImplementation(
            std::make_shared<util::DeathTestLogImplementation>());
        MutexLocker locker(mutex);
      },
      ".*previous owner of mutex [^ ]+ died.*");

  mutex->~Mutex();
}

TEST_F(IPCMutexLockerTest, Basic) {
  {
    aos::IPCMutexLocker locker(&test_mutex_);
    EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());
    EXPECT_FALSE(locker.owner_died());
  }
  EXPECT_EQ(Mutex::State::kLocked, test_mutex_.TryLock());

  test_mutex_.Unlock();
}

// Tests what happens when the caller doesn't check if the previous owner died
// with an IPCMutexLocker.
TEST_F(IPCMutexLockerDeathTest, NoCheckOwnerDied) {
  EXPECT_DEATH({ aos::IPCMutexLocker locker(&test_mutex_); },
               "nobody checked if the previous owner of mutex [^ ]+ died.*");
}

TEST_F(IPCRecursiveMutexLockerTest, Basic) {
  {
    aos::IPCRecursiveMutexLocker locker(&test_mutex_);
    EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());
    EXPECT_FALSE(locker.owner_died());
  }
  EXPECT_EQ(Mutex::State::kLocked, test_mutex_.TryLock());

  test_mutex_.Unlock();
}

// Tests actually locking a mutex recursively with IPCRecursiveMutexLocker.
TEST_F(IPCRecursiveMutexLockerTest, RecursiveLock) {
  {
    aos::IPCRecursiveMutexLocker locker(&test_mutex_);
    EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());
    {
      aos::IPCRecursiveMutexLocker locker(&test_mutex_);
      EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());
      EXPECT_FALSE(locker.owner_died());
    }
    EXPECT_EQ(Mutex::State::kLockFailed, test_mutex_.TryLock());
    EXPECT_FALSE(locker.owner_died());
  }
  EXPECT_EQ(Mutex::State::kLocked, test_mutex_.TryLock());

  test_mutex_.Unlock();
}

// Tests that IPCMutexLocker behaves correctly when the previous owner dies.
TEST_F(IPCMutexLockerTest, OwnerDied) {
  testing::TestSharedMemory my_shm;
  Mutex *mutex =
      static_cast<Mutex *>(shm_malloc_aligned(sizeof(Mutex), alignof(Mutex)));
  new (mutex) Mutex();

  std::thread thread([&]() { ASSERT_FALSE(mutex->Lock()); });
  thread.join();
  {
    aos::IPCMutexLocker locker(mutex);
    EXPECT_EQ(Mutex::State::kLockFailed, mutex->TryLock());
    EXPECT_TRUE(locker.owner_died());
  }
  EXPECT_EQ(Mutex::State::kLocked, mutex->TryLock());

  mutex->Unlock();
  mutex->~Mutex();
}

}  // namespace testing
}  // namespace aos
