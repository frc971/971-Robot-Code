#ifndef AOS_IPC_LIB_LATENCY_LIB_H_
#define AOS_IPC_LIB_LATENCY_LIB_H_

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <chrono>

#include "aos/time/time.h"
#include "glog/logging.h"

namespace aos {

void TimerThread(monotonic_clock::time_point end_time, int timer_priority);

class Tracing {
 public:
  Tracing() {
    SetContentsOrDie("/sys/kernel/debug/tracing/events/enable", "1\n");
    fd_ = open("/sys/kernel/debug/tracing/tracing_on",
               O_WRONLY | O_TRUNC | O_CLOEXEC, 0);
    PCHECK(fd_);
  }

  ~Tracing() { close(fd_); }

  void Start() { PCHECK(write(fd_, "1\n", 2)); }

  void Stop() { PCHECK(write(fd_, "0\n", 2)); }

 private:
  void SetContentsOrDie(const char *filename, const char *data) {
    int fd = open(filename, O_WRONLY | O_TRUNC | O_CLOEXEC);
    PCHECK(fd);
    PCHECK(write(fd, data, strlen(data)));
    PCHECK(close(fd));
  }

  int fd_;
};

}  // namespace aos

#endif  // AOS_IPC_LIB_LATENCY_LIB_H_
