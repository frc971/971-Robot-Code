#include "aos/common/condition.h"

#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#include "gtest/gtest.h"

#include "aos/common/time.h"
#include "aos/common/mutex.h"
#include "aos/common/queue_testutils.h"
#include "aos/common/type_traits.h"
#include "aos/linux_code/ipc_lib/core_lib.h"
#include "aos/common/logging/logging.h"
#include "aos/common/macros.h"

using ::aos::time::Time;
using ::aos::common::testing::GlobalCoreInstance;

namespace aos {
namespace testing {

class ConditionTest : public ::testing::Test {
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

  GlobalCoreInstance my_core;

  Shared *const shared_;

  void Settle() {
    time::SleepFor(::Time::InSeconds(0.008));
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
  static constexpr ::Time kMinimumDelay = ::Time::InSeconds(0.015);
  static constexpr ::Time kDefaultTimeout = ::Time::InSeconds(0.09);

  // delay is how long to wait before doing action to condition.
  // timeout is how long to wait after delay before deciding that it's hung.
  ConditionTestProcess(const ::Time &delay, Action action, Condition *condition,
                       const ::Time &timeout = kDefaultTimeout)
    : delay_(kMinimumDelay + delay), action_(action), condition_(condition),
      timeout_(delay_ + timeout), child_(-1),
      shared_(static_cast<Shared *>(shm_malloc(sizeof(Shared)))) {
    new (shared_) Shared();
  }
  ~ConditionTestProcess() {
    assert(child_ == -1);
  }

  void Start() {
    ASSERT_FALSE(shared_->started);

    child_ = fork();
    if (child_ == 0) {  // in child
      ::aos::common::testing::PreventExit();
      Run();
      exit(EXIT_SUCCESS);
    } else {  // in parent
      assert(child_ != -1);

      shared_->ready.Lock();

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
      if (shared_->start_time > ::Time::Now() + timeout_) {
        Kill();
        return ::testing::AssertionSuccess() << "already been too long";
      }
    } else {
      shared_->done_delaying.Lock();
    }
    time::SleepFor(::Time::InSeconds(0.01));
    if (!shared_->finished) time::SleepUntil(shared_->start_time + timeout_);
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
      : started(false), delayed(false), start_time(0, 0), finished(false) {
      done_delaying.Lock();
      ready.Lock();
    }

    volatile bool started;
    volatile bool delayed;
    Mutex done_delaying;
    ::Time start_time;
    volatile bool finished;
    Mutex ready;
  };
  static_assert(shm_ok<Shared>::value,
                "it's going to get shared between forked processes");

  void Run() {
    if (action_ == Action::kWaitLockStart) {
      shared_->ready.Unlock();
      condition_->m()->Lock();
    }
    time::SleepFor(delay_);
    shared_->start_time = ::Time::Now();
    shared_->delayed = true;
    shared_->done_delaying.Unlock();
    if (action_ != Action::kWaitLockStart) {
      shared_->ready.Unlock();
      condition_->m()->Lock();
    }
    condition_->Wait();
    shared_->finished = true;
    if (action_ != Action::kWaitNoUnlock) {
      condition_->m()->Unlock();
    }
  }

  void Join() {
    assert(child_ != -1);
    int status;
    do {
      assert(waitpid(child_, &status, 0) == child_);
    } while (!(WIFEXITED(status) || WIFSIGNALED(status)));
    child_ = -1;
  }
  void Kill() {
    assert(child_ != -1);
    assert(kill(child_, SIGTERM) == 0);
    Join();
  }

  const ::Time delay_;
  const Action action_;
  Condition *const condition_;
  const ::Time timeout_;

  pid_t child_;

  Shared *const shared_;

  DISALLOW_COPY_AND_ASSIGN(ConditionTestProcess);
};
constexpr ::Time ConditionTestProcess::kMinimumDelay;
constexpr ::Time ConditionTestProcess::kDefaultTimeout;

// Makes sure that the testing framework and everything work for a really simple
// Wait() and then Signal().
TEST_F(ConditionTest, Basic) {
  ConditionTestProcess child(::Time(0, 0),
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
  ConditionTestProcess child(::Time(0, 0),
                             ConditionTestProcess::Action::kWait,
                             &shared_->condition);
  shared_->mutex.Lock();
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
  ConditionTestProcess child(::Time(0, 0),
                             ConditionTestProcess::Action::kWait,
                             &shared_->condition);
  shared_->mutex.Lock();
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
  ConditionTestProcess child(::Time(0, 0),
                             ConditionTestProcess::Action::kWaitNoUnlock,
                             &shared_->condition);
  child.Start();
  Settle();
  shared_->condition.Signal();
  EXPECT_FALSE(child.Hung());
  EXPECT_FALSE(shared_->mutex.TryLock());
}

// Tests that Signal() stops exactly 1 Wait()er.
TEST_F(ConditionTest, SignalOne) {
  ConditionTestProcess child1(::Time(0, 0),
                              ConditionTestProcess::Action::kWait,
                              &shared_->condition);
  ConditionTestProcess child2(::Time(0, 0),
                              ConditionTestProcess::Action::kWait,
                              &shared_->condition);
  ConditionTestProcess child3(::Time(0, 0),
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
  ConditionTestProcess child1(::Time(0, 0),
                              ConditionTestProcess::Action::kWait,
                              &shared_->condition);
  ConditionTestProcess child2(::Time(0, 0),
                              ConditionTestProcess::Action::kWait,
                              &shared_->condition);
  ConditionTestProcess child3(::Time(0, 0),
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
