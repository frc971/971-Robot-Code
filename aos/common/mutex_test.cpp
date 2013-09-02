#include "aos/common/mutex.h"

#include <sched.h>
#include <math.h>
#include <pthread.h>
#ifdef __VXWORKS__
#include <taskLib.h>
#endif

#include "gtest/gtest.h"

#include "aos/atom_code/ipc_lib/aos_sync.h"

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
TEST_F(MutexTest, MutexUnlocker) {
  test_mutex.Lock();
  {
    aos::MutexUnlocker unlocker(&test_mutex);
    // If this fails, then something weird is going on and the next line might
    // hang.
    ASSERT_TRUE(test_mutex.TryLock());
    test_mutex.Unlock();
  }
  EXPECT_FALSE(test_mutex.TryLock());
}

// A worker thread for testing the fairness of the mutex implementation.
class MutexFairnessWorkerThread {
 public:
  MutexFairnessWorkerThread(int *cycles, int index,
                            Mutex *in_mutex, mutex *start)
      : cycles_(cycles), index_(index), mutex_(in_mutex), start_(start) {}

  static void *RunStatic(void *self_in) {
    MutexFairnessWorkerThread *self =
        static_cast<MutexFairnessWorkerThread *>(self_in);
    self->Run();
    delete self;
    return NULL;
  }

  static void Reset(int cycles) {
    cyclesRun = 0;
    totalCycles = cycles;
  }

 private:
  void Run() {
    cycles_[index_] = 0;
    ASSERT_EQ(futex_wait(start_), 0);
    while (cyclesRun < totalCycles) {
      {
        MutexLocker locker(mutex_);
        ++cyclesRun;
      }
      ++cycles_[index_];
      // Otherwise the fitpc implementation tends to just relock in the same
      // thread.
      sched_yield();
    }

#ifdef __VXWORKS__
    // Without this, all of the "task ... deleted ..." messages come out at
    // once, and it looks weird and triggers an socat bug (at least for
    // Squeeze's version 1.7.1.3-1).
    taskDelay(index_);
#endif
  }

  int *cycles_;
  int index_;
  Mutex *mutex_;
  mutex *start_;
  static int cyclesRun, totalCycles;
};
int MutexFairnessWorkerThread::cyclesRun;
int MutexFairnessWorkerThread::totalCycles;
// Tests the fairness of the implementation. It does this by repeatedly locking
// and unlocking a mutex in multiple threads and then checking the standard
// deviation of the number of times each one locks.
//
// It is safe to do this with threads because this is the test so it can change
// if the implementations ever change to not support that. Fitpc logging calls
// are not thread-safe, but it doesn't really matter because the only logging
// call that would get made would be a LOG(FATAL) that would still terminate the
// process.
TEST_F(MutexTest, Fairness) {
  static const int kThreads = 13;
#ifdef __VXWORKS__
  static const int kWarmupCycles = 1000, kRunCycles = 60000, kMaxDeviation = 20;
#else
  static const int kWarmupCycles = 30000, kRunCycles = 3000000, kMaxDeviation = 10000;
#endif

  int cycles[kThreads];
  pthread_t workers[kThreads];
  mutex start = 0;

  for (int repeats = 0; repeats < 2; ++repeats) {
    futex_unset(&start);
    MutexFairnessWorkerThread::Reset(repeats ? kRunCycles : kWarmupCycles);
    for (int i = 0; i < kThreads; ++i) {
      MutexFairnessWorkerThread *c = new MutexFairnessWorkerThread(cycles, i,
                                                                   &test_mutex,
                                                                   &start);
      ASSERT_EQ(0, pthread_create(&workers[i], NULL,
                                  MutexFairnessWorkerThread::RunStatic, c));
    }
    futex_set(&start);
    for (int i = 0; i < kThreads; ++i) {
      ASSERT_EQ(0, pthread_join(workers[i], NULL));
    }
  }

  double variance = 0;
  int expected = kRunCycles / kThreads;
  for (int i = 0; i < kThreads; ++i) {
    variance += (cycles[i] - expected) * (cycles[i] - expected);
  }
  double deviation = sqrt(variance / kThreads);
  printf("deviation=%f\n", deviation);
  ASSERT_GT(deviation, 0);
  EXPECT_LT(deviation, kMaxDeviation);
}

}  // namespace testing
}  // namespace aos
