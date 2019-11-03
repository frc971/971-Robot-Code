#ifndef AOS_IPC_LIB_SIGNALFD_H_
#define AOS_IPC_LIB_SIGNALFD_H_

#include <sys/signalfd.h>
#include <sys/types.h>
#include <initializer_list>

namespace aos {
namespace ipc_lib {

// Class to manage a signalfd.
class SignalFd {
 public:
  // Constructs a SignalFd for the provided list of signals.
  // Blocks the signals at the same time in this thread.
  SignalFd(::std::initializer_list<unsigned int> signals);

  SignalFd(const SignalFd &) = delete;
  SignalFd &operator=(const SignalFd &) = delete;
  ~SignalFd();

  // Returns the file descriptor for the signalfd.
  int fd() { return fd_; }

  // Reads a signalfd_siginfo.  If there was an error, the resulting ssi_signo
  // will be 0.
  signalfd_siginfo Read();

 private:
  int fd_ = -1;

  // The signals we blocked in the constructor.
  sigset_t blocked_mask_;
};

}  // namespace ipc_lib
}  // namespace aos

#endif  // AOS_IPC_LIB_SIGNALFD_H_
