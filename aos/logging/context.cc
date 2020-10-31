#include "aos/logging/context.h"

#ifndef _GNU_SOURCE
#define _GNU_SOURCE /* See feature_test_macros(7) */
#endif

#if __has_feature(memory_sanitizer)
#include <sanitizer/msan_interface.h>
#endif
#include <string.h>
#include <sys/prctl.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>

#include <string>

extern char *program_invocation_name;
extern char *program_invocation_short_name;

#include "glog/logging.h"

#include "aos/complex_thread_local.h"
#include "aos/die.h"
#include "aos/logging/implementations.h"
#include "aos/thread_local.h"

namespace aos {
namespace logging {
namespace internal {
namespace {

// TODO(brians): Differentiate between threads with the same name in the same
// process.

::std::string GetMyName() {
  // The maximum number of characters that can make up a thread name.
  // The docs are unclear if it can be 16 characters with no '\0', so we'll be
  // safe by adding our own where necessary.
  static const size_t kThreadNameLength = 16;

  ::std::string process_name(program_invocation_short_name);

  char thread_name_array[kThreadNameLength + 1];
  if (prctl(PR_GET_NAME, thread_name_array) != 0) {
    PLOG(FATAL) << "prctl(PR_GET_NAME, " << thread_name_array << ") failed";
  }
#if __has_feature(memory_sanitizer)
  // msan doesn't understand PR_GET_NAME, so help it along.
  __msan_unpoison(thread_name_array, sizeof(thread_name_array));
#endif
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

::aos::ComplexThreadLocal<Context> my_context;

// True if we're going to delete the current Context object ASAP. The
// reason for doing this instead of just deleting them is that tsan (at least)
// doesn't like it when pthread_atfork handlers do complicated stuff and it's
// not a great idea anyways.
AOS_THREAD_LOCAL bool delete_current_context(false);

}  // namespace

Context::Context() : sequence(0) {}

// Used in aos/linux_code/init.cc when a thread's name is changed.
void ReloadThreadName() {
  if (my_context.created()) {
    ::std::string my_name = GetMyName();
    if (my_name.size() + 1 > sizeof(Context::name)) {
      Die("logging: process/thread name '%s' is too long\n", my_name.c_str());
    }
    strcpy(my_context->name, my_name.c_str());
    my_context->name_size = my_name.size();
  }
}

Context *Context::Get() {
  if (__builtin_expect(delete_current_context, false)) {
    my_context.Clear();
    delete_current_context = false;
  }
  if (__builtin_expect(!my_context.created(), false)) {
    my_context.Create();
    ReloadThreadName();
    my_context->source = getpid();
  }
  return my_context.get();
}

void Context::Delete() { delete_current_context = true; }

void Context::DeleteNow() {
  my_context.Clear();
  delete_current_context = false;
}

}  // namespace internal
}  // namespace logging
}  // namespace aos
