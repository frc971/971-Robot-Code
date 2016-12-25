#include "aos/common/condition.h"

#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "gtest/gtest.h"

#include "aos/common/time.h"
#include "aos/common/mutex.h"
#include "aos/testing/test_shm.h"
#include "aos/common/type_traits.h"
#include "aos/linux_code/ipc_lib/core_lib.h"
#include "aos/common/logging/logging.h"
#include "aos/common/macros.h"
#include "aos/linux_code/ipc_lib/aos_sync.h"
#include "aos/common/die.h"
#include "aos/common/util/thread.h"
#include "aos/testing/prevent_exit.h"

namespace aos {
namespace testing {

namespace chrono = ::std::chrono;

class ConditionTestCommon : public ::testing::Test {
 public:
  ConditionTestCommon() {}

  void Settle() { ::std::this_thread::sleep_for(chrono::milliseconds(8)); }

 private:
  DISALLOW_COPY_AND_ASSIGN(ConditionTestCommon);
};

// Some simple tests that don't rely on a TestSharedMemory to help with
// debugging problems that cause tests using that to just completely lock up.
class SimpleConditionTest : public ConditionTestCommon {
 public:
  SimpleConditionTest() : condition_(&mutex_) {}

  Mutex mutex_;
  Condition condition_;

 private:
  DISALLOW_COPY_AND_ASSIGN(SimpleConditionTest);
};

// Makes sure that nothing crashes or anything with a basic Wait() and then
// Signal().
// This test is written to hopefully fail instead of deadlocking on failure, but
// it's tricky because there's no way to kill the child in the middle.
TEST_F(SimpleConditionTest, Basic) {
  ::std::atomic_bool child_finished(false);
  Condition child_ready(&mutex_);
  ASSERT_FALSE(mutex_.Lock());
  util::FunctionThread child([this, &child_finished, &child_ready](
      util::FunctionThread *) {
    ASSERT_FALSE(mutex_.Lock());
    child_ready.Broadcast();
    ASSERT_FALSE(condition_.Wait());
    child_finished.store(true);
    mutex_.Unlock();
  });
  child.Start();
  ASSERT_FALSE(child_ready.Wait());
  EXPECT_FALSE(child_finished.load());
  condition_.Signal();
  mutex_.Unlock();
  child.Join();
  EXPECT_TRUE(child_finished.load());
}

// Tests that contention on the associated mutex doesn't break anything.
// This seems likely to cause issues with AddressSanitizer in particular.
TEST_F(SimpleConditionTest, MutexContention) {
  for (int i = 0; i < 1000; ++i) {
    ASSERT_FALSE(mutex_.Lock());
    ::std::thread thread([this]() {
      ASSERT_FALSE(mutex_.Lock());
      condition_.Signal();
      mutex_.Unlock();
    });
    ASSERT_FALSE(condition_.Wait());
    mutex_.Unlock();
    thread.join();
  }
}

class ConditionTest : public ConditionTestCommon {
 public:
  struct Shared {
    Shared() : condition(&mutex) {}

    Mutex mutex;
    Condition condition;
  };
  static_assert(shm_ok<Shared>::value,
                "it's going to get shared between forked processes");

  ConditionTest() : shared_(static_cast<Shared *>(shm_malloc(sizeof(Shared)))) {
    new (shared_) Shared();
  }
  ~ConditionTest() {
    shared_->~Shared();
  }

  ::aos::testing::TestSharedMemory my_shm_;

  Shared *const shared_;

 protected:
  void SetUp() override {
    SetDieTestMode(true);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(ConditionTest);
};

class ConditionTestProcess {
 public:
  enum class Action {
    kWaitLockStart,  // lock, delay, wait, unlock
    kWait,  // delay, lock, wait, unlock
    kWaitNoUnlock,  // delay, lock, wait
  };

  // This amount gets added to any passed in delay to make the test repeatable.
  static constexpr chrono::milliseconds kMinimumDelay =
      chrono::milliseconds(150);
  static constexpr chrono::milliseconds kDefaultTimeout =
      chrono::milliseconds(150);

  // delay is how long to wait before doing action to condition.
  // timeout is how long to wait after delay before deciding that it's hung.
  ConditionTestProcess(chrono::milliseconds delay, Action action,
                       Condition *condition,
                       chrono::milliseconds timeout = kDefaultTimeout)
      : delay_(kMinimumDelay + delay),
        action_(action),
        condition_(condition),
        timeout_(delay_ + timeout),
        child_(-1),
        shared_(static_cast<Shared *>(shm_malloc(sizeof(Shared)))) {
    new (shared_) Shared();
  }
  ~ConditionTestProcess() {
    CHECK_EQ(child_, -1);
  }

  void Start() {
    ASSERT_FALSE(shared_->started);

    child_ = fork();
    if (child_ == 0) {  // in child
      ::aos::testing::PreventExit();
      Run();
      exit(EXIT_SUCCESS);
    } else {  // in parent
      CHECK_NE(child_, -1);

      ASSERT_EQ(0, futex_wait(&shared_->ready));

      shared_->started = true;
    }
  }

  bool IsFinished() {
    return shared_->finished;
  }

  ::testing::AssertionResult Hung() {
    if (!shared_->started) {
      ADD_FAILURE();
      return ::testing::AssertionFailure() << "not started yet";
    }
    if (shared_->finished) {
      Join();
      return ::testing::AssertionFailure() << "already returned";
    }
    if (shared_->delayed) {
      if (shared_->start_time > monotonic_clock::now() + timeout_) {
        Kill();
        return ::testing::AssertionSuccess() << "already been too long";
      }
    } else {
      CHECK_EQ(0, futex_wait(&shared_->done_delaying));
    }
    ::std::this_thread::sleep_for(chrono::milliseconds(10));
    if (!shared_->finished) {
      ::std::this_thread::sleep_until(shared_->start_time + timeout_);
    }
    if (shared_->finished) {
      Join();
      return ::testing::AssertionFailure() << "completed within timeout";
    } else {
      Kill();
      return ::testing::AssertionSuccess() << "took too long";
    }
  }
  ::testing::AssertionResult Test() {
    Start();
    return Hung();
  }

 private:
  struct Shared {
    Shared()
        : started(false),
          delayed(false),
          done_delaying(0),
          start_time(monotonic_clock::epoch()),
          finished(false),
          ready(0) {}

    volatile bool started;
    volatile bool delayed;
    aos_futex done_delaying;
    monotonic_clock::time_point start_time;
    volatile bool finished;
    aos_futex ready;
  };
  static_assert(shm_ok<Shared>::value,
                "it's going to get shared between forked processes");

  void Run() {
    if (action_ == Action::kWaitLockStart) {
      ASSERT_EQ(1, futex_set(&shared_->ready));
      ASSERT_FALSE(condition_->m()->Lock());
    }
    ::std::this_thread::sleep_for(delay_);
    shared_->start_time = monotonic_clock::now();
    shared_->delayed = true;
    ASSERT_NE(-1, futex_set(&shared_->done_delaying));
    if (action_ != Action::kWaitLockStart) {
      ASSERT_EQ(1, futex_set(&shared_->ready));
      ASSERT_FALSE(condition_->m()->Lock());
    }
    // TODO(brians): Test this returning true (aka the owner dying).
    ASSERT_FALSE(condition_->Wait());
    shared_->finished = true;
    if (action_ != Action::kWaitNoUnlock) {
      condition_->m()->Unlock();
    }
  }

  void Join() {
    CHECK_NE(child_, -1);
    int status;
    do {
      CHECK_EQ(waitpid(child_, &status, 0), child_);
    } while (!(WIFEXITED(status) || WIFSIGNALED(status)));
    child_ = -1;
  }
  void Kill() {
    CHECK_NE(child_, -1);
    PCHECK(kill(child_, SIGTERM));
    Join();
  }

  const chrono::milliseconds delay_;
  const Action action_;
  Condition *const condition_;
  const chrono::milliseconds timeout_;

  pid_t child_;

  Shared *const shared_;

  DISALLOW_COPY_AND_ASSIGN(ConditionTestProcess);
};
constexpr chrono::milliseconds ConditionTestProcess::kMinimumDelay;
constexpr chrono::milliseconds ConditionTestProcess::kDefaultTimeout;

// Makes sure that the testing framework and everything work for a really simple
// Wait() and then Signal().
TEST_F(ConditionTest, Basic) {
  ConditionTestProcess child(chrono::milliseconds(0),
                             ConditionTestProcess::Action::kWait,
                             &shared_->condition);
  child.Start();
  Settle();
  EXPECT_FALSE(child.IsFinished());
  shared_->condition.Signal();
  EXPECT_FALSE(child.Hung());
}

// Makes sure that the worker child locks before it tries to Wait() etc.
TEST_F(ConditionTest, Locking) {
  ConditionTestProcess child(chrono::milliseconds(0),
                             ConditionTestProcess::Action::kWait,
                             &shared_->condition);
  ASSERT_FALSE(shared_->mutex.Lock());
  child.Start();
  Settle();
  // This Signal() shouldn't do anything because the child should still be
  // waiting to lock the mutex.
  shared_->condition.Signal();
  Settle();
  shared_->mutex.Unlock();
  EXPECT_TRUE(child.Hung());
}

// Tests that the work child only catches a Signal() after the mutex gets
// unlocked.
TEST_F(ConditionTest, LockFirst) {
  ConditionTestProcess child(chrono::milliseconds(0),
                             ConditionTestProcess::Action::kWait,
                             &shared_->condition);
  ASSERT_FALSE(shared_->mutex.Lock());
  child.Start();
  Settle();
  shared_->condition.Signal();
  Settle();
  EXPECT_FALSE(child.IsFinished());
  shared_->mutex.Unlock();
  Settle();
  EXPECT_FALSE(child.IsFinished());
  shared_->condition.Signal();
  EXPECT_FALSE(child.Hung());
}

// Tests that the mutex gets relocked after Wait() returns.
TEST_F(ConditionTest, Relocking) {
  ConditionTestProcess child(chrono::milliseconds(0),
                             ConditionTestProcess::Action::kWaitNoUnlock,
                             &shared_->condition);
  child.Start();
  Settle();
  shared_->condition.Signal();
  EXPECT_FALSE(child.Hung());
  EXPECT_EQ(Mutex::State::kOwnerDied, shared_->mutex.TryLock());
  shared_->mutex.Unlock();
}

// Tests that Signal() stops exactly 1 Wait()er.
TEST_F(ConditionTest, SignalOne) {
  ConditionTestProcess child1(chrono::milliseconds(0),
                              ConditionTestProcess::Action::kWait,
                              &shared_->condition);
  ConditionTestProcess child2(chrono::milliseconds(0),
                              ConditionTestProcess::Action::kWait,
                              &shared_->condition);
  ConditionTestProcess child3(chrono::milliseconds(0),
                              ConditionTestProcess::Action::kWait,
                              &shared_->condition);
  auto number_finished = [&]() { return (child1.IsFinished() ? 1 : 0) +
    (child2.IsFinished() ? 1 : 0) + (child3.IsFinished() ? 1 : 0); };
  child1.Start();
  child2.Start();
  child3.Start();
  Settle();
  EXPECT_EQ(0, number_finished());
  shared_->condition.Signal();
  Settle();
  EXPECT_EQ(1, number_finished());
  shared_->condition.Signal();
  Settle();
  EXPECT_EQ(2, number_finished());
  shared_->condition.Signal();
  Settle();
  EXPECT_EQ(3, number_finished());
  EXPECT_FALSE(child1.Hung());
  EXPECT_FALSE(child2.Hung());
  EXPECT_FALSE(child3.Hung());
}

// Tests that Brodcast() wakes multiple Wait()ers.
TEST_F(ConditionTest, Broadcast) {
  ConditionTestProcess child1(chrono::milliseconds(0),
                              ConditionTestProcess::Action::kWait,
                              &shared_->condition);
  ConditionTestProcess child2(chrono::milliseconds(0),
                              ConditionTestProcess::Action::kWait,
                              &shared_->condition);
  ConditionTestProcess child3(chrono::milliseconds(0),
                              ConditionTestProcess::Action::kWait,
                              &shared_->condition);
  child1.Start();
  child2.Start();
  child3.Start();
  Settle();
  shared_->condition.Broadcast();
  EXPECT_FALSE(child1.Hung());
  EXPECT_FALSE(child2.Hung());
  EXPECT_FALSE(child3.Hung());
}

}  // namespace testing
}  // namespace aos
