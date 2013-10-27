#include "aos/atom_code/logging/atom_logging.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>
#include <unistd.h>
#include <limits.h>
#include <sys/prctl.h>

#include <algorithm>

#include "aos/common/die.h"
#include "aos/common/logging/logging_impl.h"
#include "aos/atom_code/thread_local.h"
#include "aos/atom_code/ipc_lib/queue.h"

namespace aos {
namespace logging {
namespace {

using internal::Context;

AOS_THREAD_LOCAL Context *my_context(NULL);

::std::string GetMyName() {
  // The maximum number of characters that can make up a thread name.
  // The docs are unclear if it can be 16 characters with no '\0', so we'll be
  // safe by adding our own where necessary.
  static const size_t kThreadNameLength = 16;

  ::std::string process_name(program_invocation_short_name);

  char thread_name_array[kThreadNameLength + 1];
  if (prctl(PR_GET_NAME, thread_name_array) != 0) {
    Die("prctl(PR_GET_NAME, %p) failed with %d: %s\n",
        thread_name_array, errno, strerror(errno));
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

RawQueue *queue;

}  // namespace
namespace internal {

Context *Context::Get() {
  if (my_context == NULL) {
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
  delete my_context;
  my_context = NULL;
}

}  // namespace internal
namespace atom {
namespace {

class AtomQueueLogImplementation : public LogImplementation {
  virtual void DoLog(log_level level, const char *format, va_list ap) {
    LogMessage *message = static_cast<LogMessage *>(queue->GetMessage());
    if (message == NULL) {
      LOG(FATAL, "queue get message failed\n");
    }

    internal::FillInMessage(level, format, ap, message);

    Write(message);
  }
};

}  // namespace

void Register() {
  Init();

  queue = RawQueue::Fetch("LoggingQueue", sizeof(LogMessage), 1323, 1500);
  if (queue == NULL) {
    Die("logging: couldn't fetch queue\n");
  }

  AddImplementation(new AtomQueueLogImplementation());
}

const LogMessage *ReadNext(int flags, int *index) {
  return static_cast<const LogMessage *>(queue->ReadMessageIndex(flags, index));
}

const LogMessage *ReadNext() {
  return ReadNext(RawQueue::kBlock);
}

const LogMessage *ReadNext(int flags) {
  const LogMessage *r = NULL;
  do {
    r = static_cast<const LogMessage *>(queue->ReadMessage(flags));
    // not blocking means return a NULL if that's what it gets
  } while ((flags & RawQueue::kBlock) && r == NULL);
  return r;
}

LogMessage *Get() {
  return static_cast<LogMessage *>(queue->GetMessage());
}

void Free(const LogMessage *msg) {
  queue->FreeMessage(msg);
}

void Write(LogMessage *msg) {
  if (!queue->WriteMessage(msg, RawQueue::kOverride)) {
    LOG(FATAL, "writing failed");
  }
}

}  // namespace atom
}  // namespace logging
}  // namespace aos
