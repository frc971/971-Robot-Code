#include "aos/crio/logging/crio_logging.h"

#include <string.h>
#include <msgQLib.h>
#include <stdint.h>
#include <taskLib.h>

#include "WPILib/Timer.h"
#include "WPILib/Task.h"

#include "aos/common/logging/logging_impl.h"
#include "aos/common/network/SendSocket.h"
#include "aos/common/Configuration.h"
#include "aos/common/die.h"

namespace aos {
namespace logging {
namespace {

using internal::Context;

MSG_Q_ID queue;
static const int kQueueLength = 150;

// This gets run in a low-priority task to take the logs from the queue and send
// them to the atom over a TCP socket.
void DoTask() {
  SendSocket sock;
  LogMessage msg;
  while (true) {
    const int ret = msgQReceive(queue, reinterpret_cast<char *>(&msg),
                                sizeof(msg), WAIT_FOREVER);
    if (ret == ERROR) {
      LOG(WARNING, "receiving a message failed with %d (%s)",
          errno, strerror(errno));
      continue;
    }
    if (ret != sizeof(msg)) {
      LOG(WARNING, "received a message of size %d instead of %d\n",
          ret, sizeof(msg));
      continue;
    }

    if (sock.LastStatus() != 0) {
      int numMsgs = msgQNumMsgs(queue);
      if (numMsgs == ERROR) {
        LOG(FATAL, "msgQNumMsgs(%p) failed with %d: %s\n", queue,
            errno, strerror(errno));
      }
      // If the queue is more than a little bit full.
      // Otherwise, without a connection on the other end, the queue just fills
      // up and then all the senders start failing.
      if (numMsgs > kQueueLength * 3 / 10) {
        // DEBUG and INFO aren't worth recording here.
        if (log_gt_important(msg.level, INFO)) {
          LogNext(WARNING, "log_sender: dropping %s", msg.message);
        }
        continue;
      } else {
        if (sock.Connect(NetworkPort::kLogs,
                         configuration::GetIPAddress(
                             configuration::NetworkDevice::kAtom),
                         SOCK_STREAM) != 0) {
          LOG(WARNING, "connecting failed because of %d: %s\n",
              errno, strerror(errno));
          LogNext(WARNING, "log_sender: couldn't send %s", msg.message);
          continue;
        }
      }
    }
    sock.Send(&msg, sizeof(msg));
    if (sock.LastStatus() != 0) {
      LOG(WARNING, "sending '%s' failed because of %d: %s\n",
          msg.message, errno, strerror(errno));
    }
  }
}

// A simple linked list of Contexts.
struct ContextHolder {
  static ContextHolder *head;
  // Which task this one belongs to.
  const int task;
  ContextHolder *next;
  Context context;

  ContextHolder() : task(taskIdSelf()) {
    MutexLocker locker(&list_mutex);
    next = head;
    head = this;
  }
  ~ContextHolder() {
    MutexLocker locker(&list_mutex);
    head = next;
  }

  static ContextHolder *Get() {
    int thisTask = taskIdSelf();
    for (ContextHolder *c = head; c != NULL; c = c->next) {
      if (c->task == thisTask) {
        return c;
      }
    }
    return NULL;
  }

 private:
  static Mutex list_mutex;
};
ContextHolder *ContextHolder::head(NULL);
Mutex ContextHolder::list_mutex;

}  // namespace
namespace internal {

Context *Context::Get() {
  ContextHolder *holder = ContextHolder::Get();

  if (holder == NULL) {
    holder = new ContextHolder();

    errno = 0;  // in case taskName decides not to set it
    // We're going to make a copy of it because vxworks might allocate the
    // memory for it from some funky place or something.
    const char *my_name = taskName(0);
    if (my_name == NULL) {
      Die("logging: taskName(0) failed with %d: %s\n",
          errno, strerror(errno));
    }
    holder->context.name_size = strlen(my_name);
    if (holder->context.name_size > sizeof(LogMessage::name)) {
      Die("logging: somebody chose a task name ('%s') that's too long\n",
          my_name);
    }
    char *name_chars = new char[holder->context.name_size];
    memcpy(name_chars, my_name, holder->context.name_size);
    name_chars[holder->context.name_size - 1] = '\0';
    holder->context.name = name_chars;
    holder->context.source = taskIdSelf();
  }

  return &holder->context;
}

void Context::Delete() {
  delete ContextHolder::Get();
}

}  // namespace internal

namespace crio {
namespace {

class CrioLogImplementation : public LogImplementation {
  virtual void DoLog(log_level level, const char *format, va_list ap) {
    LogMessage message;

    internal::FillInMessage(level, format, ap, &message);

    if (msgQSend(queue, reinterpret_cast<char *>(&message), sizeof(message),
                 NO_WAIT, MSG_PRI_NORMAL) != 0) {
      if (errno == S_objLib_OBJ_UNAVAILABLE) {
        LOG(WARNING, "no space for sending %s", message.message);
      } else {
        LOG(FATAL, "msgQSend(%p, %p, %d, NO_WAIT, MSG_PRI_NORMAL) failed"
            " with %d: %s\n", queue, &message, sizeof(message),
            errno, strerror(errno));
      }
    }
  }
};

}  // namespace

void Register() {
  queue = msgQCreate(kQueueLength,
                     sizeof(LogMessage),
                     MSG_Q_PRIORITY);
  if (queue == NULL) {
    Die("msgQCreate(%d, %d, MSG_Q_PRIORITY) failed with %d: %s\n",
        kQueueLength, sizeof(LogMessage), errno, strerror(errno));
  }

  Task *task = new Task("LogSender",
                        (FUNCPTR)(DoTask),
                        150);  // low priority
  task->Start();

  Init();
  
  AddImplementation(new CrioLogImplementation());
}

}  // namespace crio
}  // namespace logging
}  // namespace aos
