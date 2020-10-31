#ifndef AOS_LOGGING_CONTEXT_H_
#define AOS_LOGGING_CONTEXT_H_

#include <inttypes.h>
#include <limits.h>
#include <stddef.h>
#include <sys/types.h>
#include <memory>

#include <atomic>

#include "aos/logging/sizes.h"

namespace aos {
namespace logging {

class LogImplementation;

// This is where all of the code that is only used by actual LogImplementations
// goes.
namespace internal {

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

  static void DeleteNow();

  // Which one to log to right now.
  // Will be NULL if there is no logging implementation to use right now and we
  // should use stderr instead.
  std::shared_ptr<LogImplementation> implementation;

  // A name representing this task/(process and thread).
  char name[LOG_MESSAGE_NAME_LEN];
  size_t name_size;

  // What to assign LogMessage::source to in this task/thread.
  pid_t source;

  // The sequence value to send out with the next message.
  uint16_t sequence;
};

}  // namespace internal
}  // namespace logging
}  // namespace aos

#endif  // AOS_LOGGING_CONTEXT_H_
