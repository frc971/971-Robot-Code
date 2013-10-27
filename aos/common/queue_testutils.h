#ifndef AOS_COMMON_QUEUE_TESTUTILS_H_
#define AOS_COMMON_QUEUE_TESTUTILS_H_

#include "aos/atom_code/ipc_lib/shared_mem.h"

// This file has some general helper functions for dealing with testing things
// that use shared memory etc.

namespace aos {
namespace common {
namespace testing {

// Manages creating and cleaning up "shared memory" which works within this
// process and any that it fork(2)s.
class GlobalCoreInstance {
 public:
  // Calls EnableTestLogging().
  GlobalCoreInstance();
  ~GlobalCoreInstance();

 private:
  struct aos_core global_core_data_;
};

// Enables the logging framework for use during a gtest test.
// It will print out all WARNING and above messages all of the time. It will
// also print out all log messages when a test fails.
// This function only needs to be called once in each process (after gtest is
// initialized), however it can be called more than that.
void EnableTestLogging();

// Registers an exit handler (using atexit(3)) which will call _exit(2).
// Intended to be called in a freshly fork(2)ed process where it will run before
// any other exit handlers that were already registered and prevent them from
// being run.
void PreventExit();

}  // namespace testing
}  // namespace common
}  // namespace aos

#endif  // AOS_COMMON_QUEUE_TESTUTILS_H_
