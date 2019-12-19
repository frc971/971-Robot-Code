#include "aos/ipc_lib/signalfd.h"

#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <initializer_list>

#include "glog/logging.h"

namespace aos {
namespace ipc_lib {

SignalFd::SignalFd(::std::initializer_list<unsigned int> signals) {
  // Build up the mask with the provided signals.
  CHECK_EQ(0, sigemptyset(&blocked_mask_));
  for (int signal : signals) {
    CHECK_EQ(0, sigaddset(&blocked_mask_, signal));
  }
  // Then build a signalfd.  Make it nonblocking so it works well with an epoll
  // loop, and have it close on exec.
  PCHECK((fd_ = signalfd(-1, &blocked_mask_, SFD_NONBLOCK | SFD_CLOEXEC)) != 0);
  // Now that we have a consumer of the signal, block the signals so the
  // signalfd gets them. Record which ones we actually blocked, so we can
  // unblock just those later.
  sigset_t old_mask;
  CHECK_EQ(0, pthread_sigmask(SIG_BLOCK, &blocked_mask_, &old_mask));
  for (int signal : signals) {
    if (sigismember(&old_mask, signal)) {
      CHECK_EQ(0, sigdelset(&blocked_mask_, signal));
    }
  }
}

SignalFd::~SignalFd() {
  // Unwind the constructor. Unblock the signals and close the fd. Verify nobody
  // else unblocked the signals we're supposed to unblock in the meantime.
  sigset_t old_mask;
  CHECK_EQ(0, pthread_sigmask(SIG_UNBLOCK, &blocked_mask_, &old_mask));
  sigset_t unblocked_mask;
  CHECK_EQ(0, sigandset(&unblocked_mask, &blocked_mask_, &old_mask));
  if (memcmp(&unblocked_mask, &blocked_mask_, sizeof(unblocked_mask)) != 0) {
    LOG(FATAL) << "Some other code unblocked one or more of our signals";
  }
  PCHECK(close(fd_) == 0);
}

signalfd_siginfo SignalFd::Read() {
  signalfd_siginfo result;

  const int ret =
      read(fd_, static_cast<void *>(&result), sizeof(signalfd_siginfo));
  // If we didn't get the right amount of data, signal that there was a problem
  // by setting the signal number to 0.
  if (ret != static_cast<int>(sizeof(signalfd_siginfo))) {
    result.ssi_signo = 0;
  } else {
    CHECK_NE(0u, result.ssi_signo);
  }
  return result;
}

}  // namespace ipc_lib
}  // namespace aos
