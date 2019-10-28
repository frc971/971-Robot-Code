#include "aos/ipc_lib/signalfd.h"

#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <initializer_list>

#include "aos/logging/logging.h"

namespace aos {
namespace ipc_lib {

SignalFd::SignalFd(::std::initializer_list<int> signals) {
  // Build up the mask with the provided signals.
  sigemptyset(&mask_);
  for (int signal : signals) {
    sigaddset(&mask_, signal);
  }
  // Then build a signalfd.  Make it nonblocking so it works well with an epoll
  // loop, and have it close on exec.
  AOS_PCHECK(fd_ = signalfd(-1, &mask_, SFD_NONBLOCK | SFD_CLOEXEC));
  // Now that we have a consumer of the signal, block the signals so the
  // signalfd gets them.
  pthread_sigmask(SIG_BLOCK, &mask_, nullptr);
}

SignalFd::~SignalFd() {
  // Unwind the constructor.  Unblock the signals and close the fd.
  pthread_sigmask(SIG_UNBLOCK, &mask_, nullptr);
  AOS_PCHECK(close(fd_));
}

signalfd_siginfo SignalFd::Read() {
  signalfd_siginfo result;

  const int ret =
      read(fd_, static_cast<void *>(&result), sizeof(signalfd_siginfo));
  // If we didn't get the right amount of data, signal that there was a problem
  // by setting the signal number to 0.
  if (ret != static_cast<int>(sizeof(signalfd_siginfo))) {
    result.ssi_signo = 0;
  }
  return result;
}

}  // namespace ipc_lib
}  // namespace aos
