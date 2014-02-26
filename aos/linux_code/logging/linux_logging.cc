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

class LinuxQueueLogImplementation : public LogImplementation {
  LogMessage *GetMessageOrDie() {
    LogMessage *message = static_cast<LogMessage *>(queue->GetMessage());
    if (message == NULL) {
      LOG(FATAL, "%p->GetMessage() failed\n", queue);
    } else {
      return message;
    }
  }

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
};

}  // namespace

void Register() {
  Init();

  queue = RawQueue::Fetch("LoggingQueue", sizeof(LogMessage), 1323, 20000);
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
  if (!queue->WriteMessage(msg, RawQueue::kNonBlock)) {
    LOG(FATAL, "writing failed\n");
  }
}

}  // namespace linux_code
}  // namespace logging
}  // namespace aos
