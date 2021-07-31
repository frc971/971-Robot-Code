#include "aos/ipc_lib/signalfd.h"

#include <signal.h>
#include <sys/types.h>
#if __has_feature(memory_sanitizer)
#include <sanitizer/msan_interface.h>
#endif
#include <unistd.h>

#include <initializer_list>

#include "glog/logging.h"

namespace aos {
namespace ipc_lib {
namespace {

// Wrapper which propagates msan information.
// TODO(Brian): Drop this once we have <https://reviews.llvm.org/D82411> to
// intercept this function natively.
int wrapped_sigandset(sigset_t *dest, const sigset_t *left,
                      const sigset_t *right) {
#if __has_feature(memory_sanitizer)
  if (left) {
    __msan_check_mem_is_initialized(left, sizeof(*left));
  }
  if (right) {
    __msan_check_mem_is_initialized(right, sizeof(*right));
  }
#endif
  const int r = sigandset(dest, left, right);
#if __has_feature(memory_sanitizer)
  if (!r && dest) {
    __msan_unpoison(dest, sizeof(*dest));
  }
#endif
  return r;
}

// Wrapper which propagates msan information.
// TODO(Brian): Drop this once we have
// <https://reviews.llvm.org/rG89ae290b58e20fc5f56b7bfae4b34e7fef06e1b1> to
// intercept this function natively.
int wrapped_pthread_sigmask(int how, const sigset_t *set, sigset_t *oldset) {
#if __has_feature(memory_sanitizer)
  if (set) {
    __msan_check_mem_is_initialized(set, sizeof(*set));
  }
#endif
  const int r = pthread_sigmask(how, set, oldset);
#if __has_feature(memory_sanitizer)
  if (!r && oldset) {
    __msan_unpoison(oldset, sizeof(*oldset));
  }
#endif
  return r;
}

}  // namespace

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
  CHECK_EQ(0, wrapped_pthread_sigmask(SIG_BLOCK, &blocked_mask_, &old_mask));
  for (int signal : signals) {
    if (sigismember(&old_mask, signal)) {
      CHECK_EQ(0, sigdelset(&blocked_mask_, signal));
    }
  }
}

namespace {
// sizeof(sigset_t) is larger than the bytes actually used to represent all
// signals. This size is only the bytes initialized. _NSIG is 1-indexed.
static constexpr size_t kSigSetSize = (_NSIG - 1) / 8;

// If the size of the mask changes, we should check that we still have
// correct behavior.
static_assert(kSigSetSize == 8 && kSigSetSize <= sizeof(sigset_t));
}  // namespace

SignalFd::~SignalFd() {
  // Unwind the constructor. Unblock the signals and close the fd. Verify nobody
  // else unblocked the signals we're supposed to unblock in the meantime.
  sigset_t old_mask;
  CHECK_EQ(0, wrapped_pthread_sigmask(SIG_UNBLOCK, &blocked_mask_, &old_mask));
  sigset_t unblocked_mask;
  CHECK_EQ(0, wrapped_sigandset(&unblocked_mask, &blocked_mask_, &old_mask));
  if (memcmp(&unblocked_mask, &blocked_mask_, kSigSetSize) != 0) {
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
