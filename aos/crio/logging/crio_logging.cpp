#include <string.h>

#include "WPILib/Timer.h"
#include "WPILib/Task.h"

#include "aos/aos_core.h"
#include "aos/common/network/SendSocket.h"
#include "aos/common/Configuration.h"
#include "aos/common/die.h"

#undef ERROR
#define DECL_LEVEL(name, value) const log_level name = value;
DECL_LEVELS
#undef DECL_LEVEL

//#define fprintf(...)

namespace aos {
namespace logging {
namespace {

MSG_Q_ID queue;
uint8_t sequence = 0;

// This gets run in a low-priority task to take the logs from the queue and send
// them to the atom over a TCP socket.
void DoTask() {
  SendSocket sock;
  log_crio_message msg;
  while (true) {
    const int ret = msgQReceive(queue, reinterpret_cast<char *>(&msg),
                                sizeof(msg), WAIT_FOREVER);
    if (ret == ERROR) {
        fprintf(stderr, "logging: warning: receiving a message failed"
                " with %d (%s)", errno, strerror(errno));
        continue;
    }
    if (ret != sizeof(msg)) {
      fprintf(stderr, "logging: warning: received a message of size %d "
              "instead of %zd\n", ret, sizeof(msg));
      continue;
    }

    if (sock.LastStatus() != 0) {
      if (sock.Connect(NetworkPort::kLogs,
                       configuration::GetIPAddress(
                           configuration::NetworkDevice::kAtom),
                       SOCK_STREAM) != 0) {
        fprintf(stderr, "logging: warning: connecting failed"
                " because of %d: %s\n", errno, strerror(errno));
      }
    }
    sock.Send(&msg, sizeof(msg));
    if (sock.LastStatus() != 0) {
      fprintf(stderr, "logging: warning: sending '%s' failed"
              " because of %d: %s\n", msg.message, errno, strerror(errno));
    }
  }
}

}  // namespace

void Start() {
  queue = msgQCreate(100,  // max messages
                     sizeof(log_crio_message),
                     MSG_Q_PRIORITY);
  Task *task = new Task("LogSender",
                        (FUNCPTR)(DoTask),
                        150);  // priority
  task->Start();
}

int Do(log_level level, const char *format, ...) {
  log_crio_message msg;
  msg.time = Timer::GetFPGATimestamp();
  msg.level = level;
  msg.sequence = __sync_fetch_and_add(&sequence, 1);

  const char *continued = "...";
  const size_t size = sizeof(msg.message) - strlen(continued);
  va_list ap;
  va_start(ap, format);
  const int ret = vsnprintf(msg.message, size, format, ap);
  va_end(ap);

  if (ret < 0) {
    fprintf(stderr, "logging: error: vsnprintf failed with %d (%s)\n",
            errno, strerror(errno));
    return -1;
  } else if (static_cast<uintmax_t>(ret) >= static_cast<uintmax_t>(size)) {
    // overwrite the NULL at the end of the existing one and
    // copy in the one on the end of continued
    memcpy(&msg.message[size - 1], continued, strlen(continued) + 1);
  }
  if (msgQSend(queue, reinterpret_cast<char *>(&msg), sizeof(msg),
               NO_WAIT, MSG_PRI_NORMAL) == ERROR) {
    fprintf(stderr, "logging: warning: sending message '%s'"
            " failed with %d (%s)", msg.message, errno, strerror(errno));
    return -1;
  }

  if (level == FATAL) {
    aos::Die("%s", msg.message);
  }

  return 0;
}

}  // namespace logging
}  // namespace aos
