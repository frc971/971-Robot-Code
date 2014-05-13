#include "aos/common/util/run_command.h"

#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#include "aos/common/logging/logging.h"

namespace aos {
namespace util {
namespace {

// RAII class to block SIGCHLD and then restore it on destruction.
class BlockSIGCHLD {
 public:
  BlockSIGCHLD() {
    sigset_t to_block;
    sigemptyset(&to_block);
    sigaddset(&to_block, SIGCHLD);
    if (sigprocmask(SIG_BLOCK, &to_block, &original_blocked_) == -1) {
      PLOG(FATAL, "sigprocmask(SIG_BLOCK, %p, %p) failed",
           &to_block, &original_blocked_);
    }
  }
  ~BlockSIGCHLD() {
    if (sigprocmask(SIG_SETMASK, &original_blocked_, nullptr) == -1) {
      PLOG(FATAL, "sigprocmask(SIG_SETMASK, %p, nullptr) failed",
           &original_blocked_);
    }
  }

 private:
  sigset_t original_blocked_;
};

}  // namespace

int RunCommand(const char *command) {
  BlockSIGCHLD blocker;
  const pid_t pid = fork();
  switch (pid) {
    case 0:  // in child
      {
        int new_stdin = open("/dev/null", O_RDONLY);
        if (new_stdin == -1) _exit(127);
        int new_stdout = open("/dev/null", O_WRONLY);
        if (new_stdout == -1) _exit(127);
        int new_stderr = open("/dev/null", O_WRONLY);
        if (new_stderr == -1) _exit(127);
        if (dup2(new_stdin, 0) != 0) _exit(127);
        if (dup2(new_stdout, 1) != 1) _exit(127);
        if (dup2(new_stderr, 2) != 2) _exit(127);
        execl("/bin/sh", "sh", "-c", command, nullptr);
        _exit(127);
      }
    case -1:
      return -1;
    default:
      int stat;
      while (waitpid(pid, &stat, 0) == -1) {
        if (errno != EINTR) {
          return -1;
        }
      }
      return stat;
  }
}

}  // namespace util
}  // namespace aos
