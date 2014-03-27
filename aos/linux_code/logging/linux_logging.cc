#include "aos/linux_code/logging/linux_logging.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>
#include <unistd.h>
#include <limits.h>
#include <inttypes.h>

#include <algorithm>

#include "aos/common/die.h"
#include "aos/common/logging/logging_impl.h"
#include "aos/linux_code/ipc_lib/queue.h"
#include "aos/common/time.h"

namespace aos {
namespace logging {
namespace linux_code {
namespace {

RawQueue *queue = NULL;

int dropped_messages = 0;
int32_t dropped_start_seconds, dropped_start_nseconds;

LogMessage *GetMessageOrDie() {
  LogMessage *message = static_cast<LogMessage *>(queue->GetMessage());
  if (message == NULL) {
    LOG(FATAL, "%p->GetMessage() failed\n", queue);
  } else {
    return message;
  }
}

class LinuxQueueLogImplementation : public LogImplementation {
  virtual void DoLog(log_level level, const char *format, va_list ap) override {
    LogMessage *message = GetMessageOrDie();
    internal::FillInMessage(level, format, ap, message);
    Write(message);
  }

  virtual void LogStruct(log_level level, const ::std::string &message_string,
                         size_t size, const MessageType *type,
                         const ::std::function<size_t(char *)> &serialize)
      override {
    LogMessage *message = GetMessageOrDie();
    internal::FillInMessageStructure(level, message_string, size, type,
                                     serialize, message);
    Write(message);
  }

  virtual void LogMatrix(log_level level, const ::std::string &message_string,
                         uint32_t type_id, int rows, int cols, const void *data)
      override {
    LogMessage *message = GetMessageOrDie();
    internal::FillInMessageMatrix(level, message_string, type_id, rows, cols,
                                  data, message);
    Write(message);
  }
};

}  // namespace

void Register() {
  Init();

  queue = RawQueue::Fetch("LoggingQueue", sizeof(LogMessage), 1323, 80000);
  if (queue == NULL) {
    Die("logging: couldn't fetch queue\n");
  }

  AddImplementation(new LinuxQueueLogImplementation());
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
  if (__builtin_expect(dropped_messages > 0, 0)) {
    LogMessage *dropped_message = GetMessageOrDie();
    internal::FillInMessageVarargs(
        ERROR, dropped_message,
        "%d logs starting at %" PRId32 ".%" PRId32 " dropped\n",
        dropped_messages, dropped_start_seconds, dropped_start_nseconds);
    if (queue->WriteMessage(dropped_message, RawQueue::kNonBlock)) {
      dropped_messages = 0;
      internal::PrintMessage(stderr, *dropped_message);
    } else {
      // Don't even bother trying to write this message because it's not likely
      // to work and it would be confusing to have one log in the middle of a
      // string of failures get through.
      ++dropped_messages;
      queue->FreeMessage(msg);
      return;
    }
  }
  if (!queue->WriteMessage(msg, RawQueue::kNonBlock)) {
    if (dropped_messages == 0) {
      dropped_start_seconds = msg->seconds;
      dropped_start_nseconds = msg->nseconds;
    }
    ++dropped_messages;
  }
}

}  // namespace linux_code
}  // namespace logging
}  // namespace aos
