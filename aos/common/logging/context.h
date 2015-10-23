#ifndef AOS_COMMON_LOGGING_CONTEXT_H_
#define AOS_COMMON_LOGGING_CONTEXT_H_

#include <inttypes.h>
#include <stddef.h>
#include <sys/types.h>
#include <limits.h>

#include <atomic>

#include "aos/common/logging/sizes.h"

namespace aos {
namespace logging {

class LogImplementation;

// This is where all of the code that is only used by actual LogImplementations
// goes.
namespace internal {

extern ::std::atomic<LogImplementation *> global_top_implementation;

// An separate instance of this class is accessible from each task/thread.
// NOTE: It will get deleted in the child of a fork.
//
// Get() and Delete() are implemented in the platform-specific interface.cc
// file.
struct Context {
  Context();

  // Gets the Context object for this task/thread. Will create one the first
  // time it is called.
  //
  // The implementation for each platform will lazily instantiate a new instance
  // and then initialize name the first time.
  // IMPORTANT: The implementation of this can not use logging.
  static Context *Get();
  // Deletes the Context object for this task/thread so that the next Get() is
  // called it will create a new one.
  // It is valid to call this when Get() has never been called.
  // This also gets called after a fork(2) in the new process, where it should
  // still work to clean up any state.
  static void Delete();

  // Which one to log to right now.
  // Will be NULL if there is no logging implementation to use right now.
  LogImplementation *implementation;

  // A name representing this task/(process and thread).
  char name[LOG_MESSAGE_NAME_LEN];
  size_t name_size;

  // What to assign LogMessage::source to in this task/thread.
  pid_t source;

  // The sequence value to send out with the next message.
  uint16_t sequence;

  // Contains all of the information related to implementing LOG_CORK and
  // LOG_UNCORK.
  struct {
    char message[LOG_MESSAGE_LEN];
    int line_min, line_max;
    // Sets the data up to record a new series of corked logs.
    void Reset() {
      message[0] = '\0';  // make strlen of it 0
      line_min = INT_MAX;
      line_max = -1;
      function = NULL;
    }
    // The function that the calls are in.
    // REMEMBER: While the compiler/linker will probably optimize all of the
    // identical strings to point to the same data, it might not, so using == to
    // compare this with another value is a bad idea.
    const char *function;
  } cork_data;
};

}  // namespace internal
}  // namespace logging
}  // namespace aos

#endif  // AOS_COMMON_LOGGING_CONTEXT_H_
