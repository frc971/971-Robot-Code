#include "aos/ipc_lib/robust_ownership_tracker.h"

#include <sys/mman.h>
#include <sys/wait.h>

#include "gtest/gtest.h"

namespace aos::ipc_lib::testing {

// Capture RobustOwnershipTracker in shared memory so it is shared across a
// fork.
class SharedRobustOwnershipTracker {
 public:
  SharedRobustOwnershipTracker() {
    tracker_ = static_cast<RobustOwnershipTracker *>(
        mmap(nullptr, sizeof(RobustOwnershipTracker), PROT_READ | PROT_WRITE,
             MAP_SHARED | MAP_ANONYMOUS, -1, 0));
    PCHECK(MAP_FAILED != tracker_);
  };
  ~SharedRobustOwnershipTracker() {
    PCHECK(munmap(tracker_, sizeof(RobustOwnershipTracker)) != -1);
  }

  // Captures the tid.
  RobustOwnershipTracker &tracker() const { return *tracker_; }

 private:
  RobustOwnershipTracker *tracker_;
};

class RobustOwnershipTrackerTest : public ::testing::Test {
 public:
  // Runs a function in a child process, and then exits afterwards.  Waits for
  // the child to finish before resuming.
  template <typename T>
  void RunInChildAndBlockUntilComplete(T fn) {
    pid_t pid = fork();
    if (pid == 0) {
      fn();
      LOG(INFO) << "Child exiting normally.";
      exit(0);
      return;
    }

    LOG(INFO) << "Child has pid " << pid;

    while (true) {
      LOG(INFO) << "Waiting for child.";
      int status;
      const pid_t waited_on = waitpid(pid, &status, 0);
      // Check for failure.
      if (waited_on == -1) {
        if (errno == EINTR) continue;
        PLOG(FATAL) << ": waitpid(" << pid << ", " << &status << ", 0) failed";
      }
      CHECK_EQ(waited_on, pid)
          << ": waitpid() got child " << waited_on << " instead of " << pid;
      CHECK(WIFEXITED(status));
      LOG(INFO) << "Status " << WEXITSTATUS(status);
      CHECK(WEXITSTATUS(status) == 0);
      return;
    }
  }

  // Returns the robust mutex.
  aos_mutex &GetMutex(RobustOwnershipTracker &tracker) {
    return tracker.mutex_;
  }

  // Returns the current start time in ticks.
  uint64_t GetStartTimeTicks(RobustOwnershipTracker &tracker) {
    return tracker.start_time_ticks_.load();
  }

  // Sets the current start time in ticks.
  void SetStartTimeTicks(RobustOwnershipTracker &tracker, uint64_t start_time) {
    tracker.start_time_ticks_ = start_time;
  }
};

// Tests that acquiring the futex doesn't erroneously report the owner (i.e.
// "us") as dead.
TEST_F(RobustOwnershipTrackerTest, AcquireWorks) {
  SharedRobustOwnershipTracker shared_tracker;

  EXPECT_FALSE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());

  // Run acquire in the this process, and expect it should not be dead until
  // after the test finishes.
  shared_tracker.tracker().Acquire();

  // We have ownership. Since we are alive, the owner should not be marked as
  // dead. We can use relaxed ordering since we are the only ones touching the
  // data here.
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_FALSE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());
}

// Tests that child death without unlocking results in the futex being marked as
// dead, and the owner being very dead.
TEST_F(RobustOwnershipTrackerTest, FutexRecovers) {
  SharedRobustOwnershipTracker shared_tracker;

  RunInChildAndBlockUntilComplete(
      [&]() { shared_tracker.tracker().Acquire(); });

  // Since the child that took ownership died, we expect that death to be
  // reported.
  EXPECT_TRUE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_TRUE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());
}

// Tests that a PID which doesn't exist results in the process being noticed as
// dead when we inspect /proc.
TEST_F(RobustOwnershipTrackerTest, NoMatchingPID) {
  SharedRobustOwnershipTracker shared_tracker;

  shared_tracker.tracker().Acquire();
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_FALSE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-attributes"
  GetMutex(shared_tracker.tracker()).futex =
      std::numeric_limits<aos_futex>::max() & FUTEX_TID_MASK;
#pragma GCC diagnostic pop

  // Since we're only pretending that the owner died (by changing the TID in the
  // futex), we only notice that the owner is dead when spending the time
  // walking through /proc.
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_TRUE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());
}

// Tests that a mismatched start time results in the process being marked as
// dead.
TEST_F(RobustOwnershipTrackerTest, NoMatchingStartTime) {
  SharedRobustOwnershipTracker shared_tracker;

  shared_tracker.tracker().Acquire();
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_FALSE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());

  EXPECT_NE(GetStartTimeTicks(shared_tracker.tracker()), 0);
  EXPECT_NE(GetStartTimeTicks(shared_tracker.tracker()),
            RobustOwnershipTracker::kNoStartTimeTicks);
  SetStartTimeTicks(shared_tracker.tracker(), 1);

  // Since we're only pretending that the owner died (by changing the tracked
  // start time ticks in the tracker), we only notice that the owner is dead
  // when spending the time walking through /proc.
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_TRUE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());
}

}  // namespace aos::ipc_lib::testing
