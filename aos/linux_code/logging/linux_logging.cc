#include "aos/linux_code/logging/linux_logging.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>
#include <unistd.h>
#include <limits.h>

#include <algorithm>

#include "aos/common/die.h"
#include "aos/common/logging/logging_impl.h"
#include "aos/linux_code/ipc_lib/queue.h"

namespace aos {
namespace logging {
namespace {

RawQueue *queue;

}  // namespace
namespace linux_code {
namespace {

class linuxQueueLogImplementation : public LogImplementation {
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

  AddImplementation(new linuxQueueLogImplementation());
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

}  // namespace linux_code
}  // namespace logging
}  // namespace aos
