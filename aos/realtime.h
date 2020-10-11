#ifndef AOS_REALTIME_H_
#define AOS_REALTIME_H_

#include <sched.h>
#include <string_view>

#include "glog/logging.h"

namespace aos {

// Locks everything into memory and sets the limits.  This plus InitNRT are
// everything you need to do before SetCurrentThreadRealtimePriority will make
// your thread RT.  Called as part of ShmEventLoop::Run()
void InitRT();

// Sets the current thread back down to non-realtime priority.
void UnsetCurrentThreadRealtimePriority();

// Sets the name of the current thread.
// This will displayed by `top -H`, dump_rtprio, and show up in logs.
// name can have a maximum of 16 characters.
void SetCurrentThreadName(const std::string_view name);

// Sets the current thread's realtime priority.
void SetCurrentThreadRealtimePriority(int priority);

// Sets the current thread's scheduling affinity.
void SetCurrentThreadAffinity(const cpu_set_t &cpuset);

// Sets up this process to write core dump files.
// This is called by Init*, but it's here for other files that want this
// behavior without calling Init*.
void WriteCoreDumps();

void LockAllMemory();

void ExpandStackSize();

// CHECKs that we are (or are not) running on the RT scheduler.  Useful for
// enforcing that operations which are or are not bounded shouldn't be run. This
// works both in simulation and when running against the real target.
void CheckRealtime();
void CheckNotRealtime();

// Marks that we are or are not running on the realtime scheduler.  Returns the
// previous state.
//
// Note: this shouldn't be used directly.  The event loop primitives should be
// used instead.
bool MarkRealtime(bool realtime);

// Class which restores the current RT state when destructed.
class ScopedRealtimeRestorer {
 public:
  ScopedRealtimeRestorer();
  ~ScopedRealtimeRestorer() { MarkRealtime(prior_); }

 private:
  const bool prior_;
};

// Class which marks us as on the RT scheduler until it goes out of scope.
// Note: this shouldn't be needed for most applications.
class ScopedRealtime {
 public:
  ScopedRealtime() : prior_(MarkRealtime(true)) {}
  ~ScopedRealtime() {
    CHECK(MarkRealtime(prior_)) << ": Priority was modified";
  }

 private:
  const bool prior_;
};

// Class which marks us as not on the RT scheduler until it goes out of scope.
// Note: this shouldn't be needed for most applications.
class ScopedNotRealtime {
 public:
  ScopedNotRealtime() : prior_(MarkRealtime(false)) {}
  ~ScopedNotRealtime() {
    CHECK(!MarkRealtime(prior_)) << ": Priority was modified";
  }

 private:
  const bool prior_;
};

}  // namespace aos

#endif  // AOS_REALTIME_H_
