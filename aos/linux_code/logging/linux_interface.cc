#include "aos/linux_code/logging/linux_logging.h"

#include <sys/prctl.h>

#include "aos/linux_code/thread_local.h"
#include "aos/common/die.h"

namespace aos {
namespace logging {
namespace internal {
namespace {

::std::string GetMyName() {
  // The maximum number of characters that can make up a thread name.
  // The docs are unclear if it can be 16 characters with no '\0', so we'll be
  // safe by adding our own where necessary.
  static const size_t kThreadNameLength = 16;

  ::std::string process_name(program_invocation_short_name);

  char thread_name_array[kThreadNameLength + 1];
  if (prctl(PR_GET_NAME, thread_name_array) != 0) {
    PDie("prctl(PR_GET_NAME, %p) failed", thread_name_array);
  }
  thread_name_array[sizeof(thread_name_array) - 1] = '\0';
  ::std::string thread_name(thread_name_array);

  // If the first bunch of characters are the same.
  // We cut off comparing at the shorter of the 2 strings because one or the
  // other often ends up cut off.
  if (strncmp(thread_name.c_str(), process_name.c_str(),
              ::std::min(thread_name.length(), process_name.length())) == 0) {
    // This thread doesn't have an actual name.
    return process_name;
  }

  return process_name + '.' + thread_name;
}

AOS_THREAD_LOCAL Context *my_context(nullptr);

// A temporary stash for Context objects that we're going to delete ASAP. The
// reason for doing this instead of just deleting them is that tsan (at least)
// doesn't like it when pthread_atfork handlers do complicated stuff and it's
// not a great idea anyways.
AOS_THREAD_LOCAL Context *old_context(nullptr);

}  // namespace

Context *Context::Get() {
  if (__builtin_expect(my_context == nullptr, 0)) {
    if (old_context != nullptr) {
      delete old_context;
      old_context = nullptr;
    }
    my_context = new Context();
    my_context->name = GetMyName();
    if (my_context->name.size() + 1 > sizeof(LogMessage::name)) {
      Die("logging: process/thread name '%s' is too long\n",
          my_context->name.c_str());
    }
    my_context->source = getpid();
  }
  return my_context;
}

void Context::Delete() {
  if (my_context != nullptr) {
    if (old_context != nullptr) {
      delete old_context;
    }
    old_context = my_context;
    my_context = nullptr;
  }
}

}  // namespace internal
}  // namespace logging
}  // namespace aos
