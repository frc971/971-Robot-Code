#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/inotify.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <signal.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <sys/wait.h>

#include <map>
#include <functional>
#include <deque>
#include <fstream>
#include <queue>
#include <list>
#include <string>
#include <vector>
#include <memory>

#include <event2/event.h>

#include "aos/common/logging/logging.h"
#include "aos/common/logging/logging_impl.h"
#include "aos/atom_code/init.h"
#include "aos/common/unique_malloc_ptr.h"
#include "aos/common/time.h"

// This is the main piece of code that starts all of the rest of the code and
// restarts it when the binaries are modified.
//
// NOTE: This executable should never exit nicely. It catches all nice attempts
// to exit, forwards them to all of the children that it has started, waits for
// them to exit nicely, and then SIGKILLs anybody left (which will always
// include itself).

using ::std::unique_ptr;

namespace aos {
namespace starter {

const char *child_list_file;

pid_t group_leader;

class EventBaseDeleter {
 public:
  void operator()(event_base *base) {
    if (base == NULL) return;
    event_base_free(base);
  }
};
typedef unique_ptr<event_base, EventBaseDeleter> EventBaseUniquePtr;
EventBaseUniquePtr libevent_base(NULL);

class EventDeleter {
 public:
  void operator()(event *evt) {
    if (evt == NULL) return;
    if (event_del(evt) != 0) {
      LOG(WARNING, "event_del(%p) failed\n", evt);
    }
  }
};
typedef unique_ptr<event, EventDeleter> EventUniquePtr;

// Watches a file path for modifications. Will clean up when destroyed.
class FileWatch {
 public:
  // Will call callback(value) when filename is modified.
  // If value is NULL, then a pointer to this object will be passed instead.
  FileWatch(std::string filename,
            std::function<void(void *)> callback, void *value,
            bool create = false, std::string check_filename = "")
      : filename_(filename), callback_(callback), value_(value),
        check_filename_(check_filename) {
    watch_ = inotify_add_watch(notify_fd, filename.c_str(),
                               create ? IN_CREATE : (IN_ATTRIB | IN_MODIFY));
    if (watch_ == -1) {
      LOG(FATAL, "inotify_add_watch(%d, %s, IN_ATTRIB | IN_MODIFY)"
          " failed with %d: %s\n",
          notify_fd, filename.c_str(), errno, strerror(errno));
    }
    watchers[watch_] = this;
  }
  // Cleans up everything.
  ~FileWatch() {
    if (watch_ != -1) {
      RemoveWatch();
    }
  }

  // After calling this method, this object won't really be doing much of
  // anything.
  void RemoveWatch() {
    assert(watch_ != -1);
    if (inotify_rm_watch(notify_fd, watch_) == -1) {
      LOG(WARNING, "inotify_rm_watch(%d, %d) failed with %d: %s\n",
          notify_fd, watch_, errno, strerror(errno));
    }
    if (watchers[watch_] != this) {
      LOG(WARNING, "watcher for %s didn't find itself in the map\n",
          filename_.c_str());
    } else {
      watchers.erase(watch_);
    }
    watch_ = -1;
  }

  // This gets called whenever the watch for this file triggers.
  void FileNotified(const char *filename) {
    if (!check_filename_.empty()) {
      if (filename == NULL) {
        return;
      }
      if (std::string(filename) != check_filename_) {
        return;
      }
    }

    callback_((value_ == NULL) ? this : value_);
  }

  // This gets set up as the callback for EV_READ on the inotify file
  // descriptor.
  static void INotifyReadable(int /*fd*/, short /*events*/, void *) {
    unsigned int to_read;
    if (ioctl(notify_fd, FIONREAD, &to_read) < 0) {
      LOG(FATAL, "FIONREAD(%d, %p) failed with %d: %s\n",
          notify_fd, &to_read, errno, strerror(errno));
    }
    inotify_event *notifyevt = static_cast<inotify_event *>(malloc(to_read));
    const char *end = reinterpret_cast<char *>(notifyevt) + to_read;
    aos::unique_c_ptr<inotify_event> freer(notifyevt);

    ssize_t ret = read(notify_fd, notifyevt, to_read);
    if (ret < 0) {
      LOG(FATAL, "read(%d, %p, %u) failed with %d: %s\n",
          notify_fd, notifyevt, to_read, errno, strerror(errno));
    }
    if (static_cast<size_t>(ret) != to_read) {
      LOG(ERROR, "read(%d, %p, %u) returned %zd instead of %u\n",
          notify_fd, notifyevt, to_read, ret, to_read);
      return;
    }

    // Might get multiple events.
    while (true) {
      if (watchers.count(notifyevt->wd) != 1) {
        LOG(ERROR, "couldn't find whose watch ID %d is\n", notifyevt->wd);
        return;
      }
      watchers[notifyevt->wd]->FileNotified((notifyevt->len > 0) ?
                                            notifyevt->name : NULL);

      notifyevt = reinterpret_cast<inotify_event *>(
          reinterpret_cast<char *>(notifyevt) +
          sizeof(*notifyevt) + notifyevt->len);
      if (reinterpret_cast<char *>(notifyevt) >= end) break;
    }
  }

  static void Init() {
    notify_fd = inotify_init1(IN_CLOEXEC);
    EventUniquePtr notify_event(event_new(libevent_base.get(), notify_fd,
                                          EV_READ | EV_PERSIST,
                                          FileWatch::INotifyReadable, NULL));
    event_add(notify_event.release(), NULL);
  }

 private:
  const std::string filename_;
  const std::function<void(void *)> callback_;
  void *const value_;
  std::string check_filename_;

  // The watch descriptor or -1 if we don't have one any more.
  int watch_;

  static std::map<int, FileWatch *> watchers;
  static int notify_fd;
};
std::map<int, FileWatch *> FileWatch::watchers;
int FileWatch::notify_fd;

std::string RunCommand(std::string command) {
  errno = 0;
  FILE *which = popen(command.c_str(), "r");
  if (which == NULL) {
    LOG(FATAL, "popen(\"%s\", \"r\") failed with %d: %s\n",
        command.c_str(), errno, strerror(errno));
  }

  size_t result_size = 128, read = 0;
  unique_c_ptr<char> result(static_cast<char *>(malloc(result_size)));
  while (true) {
    if (read == result_size) {
      result_size *= 2;
      void *new_result = realloc(result.get(), result_size);
      if (new_result == NULL) {
        LOG(FATAL, "realloc(..., %zd) failed because of %d: %s\n",
            result_size, errno, strerror(errno));
      } else {
        result.release();
        result = unique_c_ptr<char>(static_cast<char *>(new_result));
      }
    }

    size_t ret = fread(result.get() + read, 1, result_size - read, which);
    if (ret < result_size - read) {
      if (ferror(which)) {
        LOG(FATAL, "couldn't finish reading output of \"%s\"\n",
            command.c_str());
      }
    }
    read += ret;
    if (read > 0 && result.get()[read - 1] == '\n') {
      break;
    }

    if (feof(which)) {
      LOG(FATAL, "`%s` failed. didn't print anything\n", command.c_str());
    }
  }

  // Get rid of the \n and anything after it.
  *strchrnul(result.get(), '\n') = '\0';

  int child_status = pclose(which);
  if (child_status == -1) {
    LOG(FATAL, "pclose(%p) failed with %d: %s\n", which,
        errno, strerror(errno));
  }

  if (child_status != 0) {
    LOG(FATAL, "`%s` failed. return %d\n", command.c_str(), child_status);
  }

  return std::string(result.get());
}

// Will call callback(arg) after time.
void Timeout(time::Time time, void (*callback)(int, short, void *), void *arg) {
  EventUniquePtr timeout(evtimer_new(libevent_base.get(), callback, arg));
  struct timeval time_timeval = time.ToTimeval();
  evtimer_add(timeout.release(), &time_timeval);
}

// Represents a child process. It will take care of restarting itself etc.
class Child {
 public:
  // command is the (space-separated) command to run and its arguments
  Child(const std::string &command) : pid_(-1), watcher_(NULL),
        restart_timeout_(
            evtimer_new(libevent_base.get(), StaticDoRestart, this)) {
    const char *start, *end;
    start = command.c_str();
    while (true) {
      end = strchrnul(start, ' ');
      args_.push_back(std::string(start, end - start));
      start = end + 1;
      if (*end == '\0') {
        break;
      }
    }

    std::string full_binary = RunCommand("which " + args_[0]);

    binary_ = full_binary + ".stm";

    if (unlink(binary_.c_str()) != 0 && errno != ENOENT) {
      LOG(FATAL, "removing %s failed because of %d: %s\n",
          binary_.c_str(), errno, strerror(errno));
    }
    if (link(full_binary.c_str(), binary_.c_str()) != 0) {
      LOG(FATAL, "link('%s', '%s') failed because of %d: %s\n",
          full_binary.c_str(), binary_.c_str(), errno, strerror(errno));
    }

    watcher_ = unique_ptr<FileWatch>(
        new FileWatch(full_binary, StaticFileModified, this));

    Start();
  }

  pid_t pid() { return pid_; }

  // This gets called whenever the actual process dies and should (probably) be
  // restarted.
  void ProcessDied() {
    pid_ = -1;
    restarts_.push(time::Time::Now());
    if (restarts_.size() > kMaxRestartsNumber) {
      time::Time oldest = restarts_.front();
      restarts_.pop();
      if ((time::Time::Now() - oldest) > kMaxRestartsTime) {
        LOG(WARNING, "process %s getting restarted too often\n", name());
        Timeout(kResumeWait, StaticStart, this);
        return;
      }
    }
    Start();
  }

  // Returns a name for logging purposes.
  const char *name() {
    return args_[0].c_str();
  }

 private:
  struct CheckDiedStatus {
    Child *self;
    pid_t old_pid;
  };

  // How long to wait for a child to die nicely.
  static const time::Time kProcessDieTime;

  // How long to wait after the file is modified to restart it.
  // This is important because some programs like modifying the binaries by
  // writing them in little bits, which results in attempting to start partial
  // binaries without this.
  static const time::Time kRestartWaitTime;

  // It will limit restarting to kMaxRestartsNumber every kMaxRestartsTime.
  static const time::Time kMaxRestartsTime;
  static const size_t kMaxRestartsNumber = 5;
  // How long to wait if it gets restarted too many times.
  static const time::Time kResumeWait;
  // A history of the times that this process has been restarted.
  std::queue<time::Time, std::list<time::Time>> restarts_;

  pid_t pid_;

  std::deque<std::string> args_;
  std::string binary_;

  unique_ptr<FileWatch> watcher_;

  // An event that restarts after kRestartWaitTime.
  EventUniquePtr restart_timeout_;

  static void StaticFileModified(void *self) {
    static_cast<Child *>(self)->FileModified();
  }
  void FileModified() {
    struct timeval restart_time_timeval = kRestartWaitTime.ToTimeval();
    // This will reset the timeout again if it hasn't run yet.
    evtimer_add(restart_timeout_.get(), &restart_time_timeval);
  }

  static void StaticDoRestart(int, short, void *self) {
    static_cast<Child *>(self)->DoRestart();
  }

  void DoRestart() {
    if (pid_ != -1) {
      LOG(DEBUG, "sending SIGTERM to child %d to restart it\n", pid_);
      if (kill(pid_, SIGTERM) == -1) {
        LOG(WARNING, "kill(%d, SIGTERM) failed with %d: %s\n",
            pid_, errno, strerror(errno));
      }
      CheckDiedStatus *status = new CheckDiedStatus();
      status->self = this;
      status->old_pid = pid_;
      Timeout(kProcessDieTime, StaticCheckDied, status);
    }
  }

  static void StaticCheckDied(int, short, void *status_in) {
    CheckDiedStatus *status = static_cast<CheckDiedStatus *>(status_in);
    status->self->CheckDied(status->old_pid);
    delete status;
  }
  void CheckDied(pid_t old_pid) {
    if (pid_ == old_pid) {
      LOG(WARNING, "child %d refused to die\n", old_pid);
      if (kill(old_pid, SIGKILL) == -1) {
        LOG(WARNING, "kill(%d, SIGKILL) failed with %d: %s\n",
            old_pid, errno, strerror(errno));
      }
    }
  }

  static void StaticStart(int, short, void *self) {
    static_cast<Child *>(self)->Start();
  }
  void Start() {
    if (pid_ != -1) {
      LOG(WARNING, "calling Start() but already have child %d running\n",
          pid_);
      kill(pid_, SIGKILL);
      pid_ = -1;
    }
    if ((pid_ = fork()) == 0) {
      ssize_t args_size = args_.size();
      const char **argv = new const char *[args_size + 1];
      for (int i = 0; i < args_size; ++i) {
        argv[i] = args_[i].c_str();
      }
      argv[args_size] = NULL;
      // The const_cast is safe because no code that might care if it gets
      // modified can run afterwards.
      execv(binary_.c_str(), const_cast<char **>(argv));
      LOG(FATAL, "execv(%s, %p) failed with %d: %s\n",
          binary_.c_str(), argv, errno, strerror(errno));
      _exit(EXIT_FAILURE);
    }
    if (pid_ == -1) {
      LOG(WARNING, "forking to \"%s\" failed with %d: %s\n",
          binary_.c_str(), errno, strerror(errno));
    }
  }
};
const time::Time Child::kProcessDieTime = time::Time::InSeconds(0.5);
const time::Time Child::kMaxRestartsTime = time::Time::InSeconds(2);
const time::Time Child::kResumeWait = time::Time::InSeconds(1.5);
const time::Time Child::kRestartWaitTime = time::Time::InSeconds(1.5);

// This is where all of the Child instances except core live.
std::vector<unique_ptr<Child>> children;
// A global palce to hold on to which child is core.
unique_ptr<Child> core(NULL);

void kill_children(bool try_nice) {
  if (try_nice) {
    static const int kNiceStopSignal = SIGTERM;
    static const time::Time kNiceWaitTime = time::Time::InSeconds(1);

    // Make sure that we don't just nicely stop ourself...
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, kNiceStopSignal);
    sigprocmask(SIG_BLOCK, &mask, NULL);

    kill(-group_leader, kNiceStopSignal);
    time::SleepFor(kNiceWaitTime);
  }
  // Send SIGKILL to our whole process group, which will forcibly terminate any
  // of them that are still running (us for sure, maybe more too).
  kill(-group_leader, SIGKILL);
}

void exit_handler() {
  kill_children(true);
}
void signal_pg_kill_handler(int signum) {
  // If we get SIGSEGV or something who knows what's happening and we should
  // just kill everybody immediately.
  kill_children(signum == SIGHUP || signum == SIGINT || signum == SIGQUIT ||
                signum == SIGABRT || signum == SIGPIPE || signum == SIGTERM ||
                signum == SIGXCPU);
}

const unique_ptr<Child> &FindChild(pid_t pid) {
  for (auto it = children.begin(); it != children.end(); ++it) {
    if (pid == (*it)->pid()) {
      return *it;
    }
  }

  if (pid == core->pid()) {
    return core;
  }

  static const unique_ptr<Child> kNothing(NULL);
  return kNothing;
}

void sigchld_received(int /*fd*/, short events, void *) {
  if (events != EV_SIGNAL) {
    LOG(WARNING, "received an event that wasn't EV_SIGNAL\n");
    return;
  }

  // In a while loop in case we miss any SIGCHLDs.
  while (true) {
    siginfo_t infop;
    infop.si_pid = 0;
    if (waitid(P_ALL, 0, &infop, WEXITED | WSTOPPED | WNOHANG) != 0) {
      LOG(WARNING, "waitid failed with %d: %s", errno, strerror(errno));
    }
    if (infop.si_pid == 0) {
      return;  // no more child process changes pending
    }

    pid_t pid = infop.si_pid;
    int status = infop.si_status;
    const unique_ptr<Child> &child = FindChild(pid);
    if (child) {
      switch (infop.si_code) {
        case CLD_EXITED:
          LOG(WARNING, "child %d (%s) exited with status %d\n",
              pid, child->name(), status);
          break;
        case CLD_DUMPED:
          LOG(INFO, "child %d actually dumped core. "
              "falling through to killed by signal case\n", pid);
        case CLD_KILLED:
          // If somebody (possibly us) sent it SIGTERM that means that they just
          // want it to stop, so it stopping isn't a WARNING.
          LOG((status == SIGTERM) ? DEBUG : WARNING,
              "child %d (%s) was killed by signal %d (%s)\n",
              pid, child->name(), status,
              strsignal(status));
          break;
        case CLD_STOPPED:
          LOG(WARNING, "child %d (%s) was stopped by signal %d "
              "(giving it a SIGCONT(%d))\n",
              pid, child->name(), status, SIGCONT);
          kill(pid, SIGCONT);
          continue;
        default:
          LOG(WARNING, "something happened to child %d (%s) (killing it)\n",
              pid, child->name());
          kill(pid, SIGKILL);
          continue;
      }
    } else {
      LOG(WARNING, "couldn't find child for pid %d\n", pid);
    }
    if (child == core) {
      fprintf(stderr, "starter: si_code=%d CLD_EXITED=%d CLD_DUMPED=%d "
              "CLD_KILLED=%d CLD_STOPPED=%d si_status=%d (sig '%s')\n",
              infop.si_code, CLD_EXITED, CLD_DUMPED, CLD_KILLED,
              CLD_STOPPED, status, strsignal(status));
      LOG(FATAL, "core died\n");
    }

    child->ProcessDied();
  }
}

// This is the callback for when core creates the file indicating that it has
// started.
void Run(void *watch) {
  // Make it so it doesn't keep on seeing random changes in /tmp.
  static_cast<FileWatch *>(watch)->RemoveWatch();

  // It's safe now because core is up.
  aos::InitNRT();

  std::ifstream list_file(child_list_file);
  
  // while it's not at EOF
  while (true) {
    std::string child_name;
    getline(list_file, child_name);
    if ((list_file.rdstate() & std::ios_base::eofbit) != 0) {
      break;
    }
    if (list_file.rdstate() != 0) {
      LOG(FATAL, "reading input file %s failed\n", child_list_file);
    }
    children.push_back(unique_ptr<Child>(new Child(child_name)));
  }

  EventUniquePtr sigchld(event_new(libevent_base.get(), SIGCHLD,
  				   EV_SIGNAL | EV_PERSIST,
				   sigchld_received, NULL));
  event_add(sigchld.release(), NULL);
}

void Main() {
  logging::Init();
  // TODO(brians) tell logging that using the root logger from here until we
  // bring up shm is ok

  group_leader = setpgrp();

  // Make sure that we kill all children when we exit.
  atexit(exit_handler);
  // Do it on some signals too (ones that we otherwise tend to receive and then
  // leave all of our children going).
  signal(SIGHUP, signal_pg_kill_handler);
  signal(SIGINT, signal_pg_kill_handler);
  signal(SIGQUIT, signal_pg_kill_handler);
  signal(SIGILL, signal_pg_kill_handler);
  signal(SIGABRT, signal_pg_kill_handler);
  signal(SIGFPE, signal_pg_kill_handler);
  signal(SIGSEGV, signal_pg_kill_handler);
  signal(SIGPIPE, signal_pg_kill_handler);
  signal(SIGTERM, signal_pg_kill_handler);
  signal(SIGBUS, signal_pg_kill_handler);
  signal(SIGXCPU, signal_pg_kill_handler);
  
  libevent_base = EventBaseUniquePtr(event_base_new());

  FileWatch::Init();

  static const std::string kCoreTouchFileDir = "/tmp/";
  std::string core_touch_file = "starter.";
  core_touch_file += std::to_string(static_cast<intmax_t>(getpid()));
  core_touch_file += ".core_touch_file";
  FileWatch core_touch_file_watch(kCoreTouchFileDir, Run, NULL, true,
                                  core_touch_file);
  core = unique_ptr<Child>(
      new Child("core " + kCoreTouchFileDir + core_touch_file));

  FILE *pid_file = fopen("/tmp/starter.pid", "w");
  if (pid_file == NULL) {
    LOG(FATAL, "fopen(/tmp/starter.pid, w) failed with %d: %s\n",
        errno, strerror(errno));
  } else {
    if (fprintf(pid_file, "%d", core->pid()) == -1) {
      fprintf(stderr, "starter: error: fprintf(pid_file, core(=%d)) failed "
              "with %d: %s",
              core->pid(), errno, strerror(errno));
    }
    fclose(pid_file);
  }

  LOG(INFO, "waiting for %s to appear\n", core_touch_file.c_str());

  event_base_dispatch(libevent_base.get());
  LOG(FATAL, "event_base_dispatch(%p) returned\n", libevent_base.get());
}

}  // namespace starter
}  // namespace aos

int main(int argc, char *argv[]) {
  if (argc < 2) {
    fputs("starter: error: need an argument specifying what file to use\n",
          stderr);
    exit(EXIT_FAILURE);
  } else if(argc > 2) {
    fputs("starter: warning: too many arguments\n", stderr);
  }
  aos::starter::child_list_file = argv[1];

  aos::starter::Main();
}
