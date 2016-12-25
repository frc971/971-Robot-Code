// This has to come before anybody drags in <stdlib.h> or else we end up with
// the wrong version of WIFEXITED etc (for one thing, they don't const-qualify
// their casts) (sometimes at least).
#include <sys/wait.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/inotify.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <inttypes.h>

#include <map>
#include <functional>
#include <deque>
#include <fstream>
#include <queue>
#include <list>
#include <string>
#include <vector>
#include <memory>
#include <set>

#include "third_party/libevent/event.h"

#include "aos/common/logging/logging.h"
#include "aos/common/logging/implementations.h"
#include "aos/linux_code/init.h"
#include "aos/common/unique_malloc_ptr.h"
#include "aos/common/time.h"
#include "aos/common/once.h"
#include "aos/common/libc/aos_strsignal.h"
#include "aos/common/util/run_command.h"

// This is the main piece of code that starts all of the rest of the code and
// restarts it when the binaries are modified.
//
// Throughout, the code is not terribly concerned with thread safety because
// there is only 1 thread. It does some setup and then lets inotify run things
// when appropriate.
//
// NOTE: This program should never exit nicely. It catches all nice attempts to
// exit, forwards them to all of the children that it has started, waits for
// them to exit nicely, and then SIGKILLs anybody left (which will always
// include itself).

using ::std::unique_ptr;

namespace aos {
namespace starter {

namespace chrono = ::std::chrono;

// TODO(brians): split out the c++ libevent wrapper stuff into its own file(s)
class EventBaseDeleter {
 public:
  void operator()(event_base *base) {
    if (base == NULL) return;
    event_base_free(base);
  }
};
typedef unique_ptr<event_base, EventBaseDeleter> EventBaseUniquePtr;
EventBaseUniquePtr libevent_base;

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

// Watches a file path for modifications. Once created, keeps watching until
// destroyed or RemoveWatch() is called.
// TODO(brians): split this out into its own file + tests
class FileWatch {
 public:
  // Will call callback(value) when filename is modified.
  // If value is NULL, then a pointer to this object will be passed instead.
  //
  // Watching for file creations is slightly different. To do that, pass true
  // as create, the directory where the file will be created for filename, and
  // the name of the file (without directory name) for check_filename.
  FileWatch(std::string filename,
            std::function<void(void *)> callback,
            void *value,
            bool create = false,
            std::string check_filename = "")
      : filename_(filename),
        callback_(callback),
        value_(value),
        create_(create),
        check_filename_(check_filename),
        watch_(-1) {
    init_once.Get();

    CreateWatch();
  }
  // Cleans up everything.
  ~FileWatch() {
    if (watch_ != -1) {
      RemoveWatch();
    }
  }

  // After calling this method, this object won't really be doing much of
  // anything besides possibly running its callback or something.
  void RemoveWatch() {
    CHECK_NE(watch_, -1);
    CHECK_EQ(watch_to_remove_, -1);

    if (inotify_rm_watch(notify_fd, watch_) == -1) {
      PLOG(WARNING, "inotify_rm_watch(%d, %d) failed", notify_fd, watch_);
    }
    watch_to_remove_ = watch_;
    watch_ = -1;
  }

 private:
  // Performs the static initialization. Called by init_once from the
  // constructor.
  static void *Init() {
    notify_fd = inotify_init1(IN_CLOEXEC);
    EventUniquePtr notify_event(event_new(libevent_base.get(), notify_fd,
                                          EV_READ | EV_PERSIST,
                                          FileWatch::INotifyReadable, NULL));
    event_add(notify_event.release(), NULL);
    return NULL;
  }

  void RemoveWatchFromMap() {
    int watch = watch_to_remove_;
    if (watch == -1) {
      CHECK_NE(watch_, -1);
      watch = watch_;
    }
    if (watchers[watch] != this) {
      LOG(WARNING, "watcher for %s (%p) didn't find itself in the map\n",
          filename_.c_str(), this);
    } else {
      watchers.erase(watch);
    }
    LOG(DEBUG, "removed watch ID %d\n", watch);
    if (watch_to_remove_ == -1) {
      watch_ = -1;
    } else {
      watch_to_remove_ = -1;
    }
  }

  void CreateWatch() {
    CHECK_EQ(watch_, -1);
    watch_ = inotify_add_watch(notify_fd, filename_.c_str(),
                               create_ ? IN_CREATE : (IN_ATTRIB |
                                                     IN_MODIFY |
                                                     IN_DELETE_SELF |
                                                     IN_MOVE_SELF));
    if (watch_ == -1) {
      PLOG(FATAL, "inotify_add_watch(%d, %s,"
                  " %s ? IN_CREATE : (IN_ATTRIB | IN_MODIFY)) failed",
           notify_fd, filename_.c_str(), create_ ? "true" : "false");
    }
    watchers[watch_] = this;
    LOG(DEBUG, "watch for %s is %d\n", filename_.c_str(), watch_);
  }

  // This gets set up as the callback for EV_READ on the inotify file
  // descriptor. It calls FileNotified on the appropriate instance.
  static void INotifyReadable(int /*fd*/, short /*events*/, void *) {
    unsigned int to_read;
    // Use FIONREAD to figure out how many bytes there are to read.
    if (ioctl(notify_fd, FIONREAD, &to_read) < 0) {
      PLOG(FATAL, "FIONREAD(%d, %p) failed", notify_fd, &to_read);
    }
    inotify_event *notifyevt = static_cast<inotify_event *>(malloc(to_read));
    const char *end = reinterpret_cast<char *>(notifyevt) + to_read;
    aos::unique_c_ptr<inotify_event> freer(notifyevt);

    ssize_t ret = read(notify_fd, notifyevt, to_read);
    if (ret < 0) {
      PLOG(FATAL, "read(%d, %p, %u) failed", notify_fd, notifyevt, to_read);
    }
    if (static_cast<size_t>(ret) != to_read) {
      LOG(ERROR, "read(%d, %p, %u) returned %zd instead of %u\n",
          notify_fd, notifyevt, to_read, ret, to_read);
      return;
    }

    // Keep looping through until we get to the end because inotify does return
    // multiple events at once.
    while (reinterpret_cast<char *>(notifyevt) < end) {
      if (watchers.count(notifyevt->wd) != 1) {
        LOG(WARNING, "couldn't find whose watch ID %d is\n", notifyevt->wd);
      } else {
        LOG(DEBUG, "mask=%" PRIu32 "\n", notifyevt->mask);
        // If the watch was removed.
        if (notifyevt->mask & IN_IGNORED) {
          watchers[notifyevt->wd]->WatchDeleted();
        } else {
          watchers[notifyevt->wd]
              ->FileNotified((notifyevt->len > 0) ? notifyevt->name : NULL);
        }
      }

      notifyevt = reinterpret_cast<inotify_event *>(
          __builtin_assume_aligned(reinterpret_cast<char *>(notifyevt) +
                                       sizeof(*notifyevt) + notifyevt->len,
                                   alignof(inotify_event)));
    }
  }

  // INotifyReadable calls this method whenever the watch for our file gets
  // removed somehow.
  void WatchDeleted() {
    LOG(DEBUG, "watch for %s deleted\n", filename_.c_str());
    RemoveWatchFromMap();
    CreateWatch();
  }

  // INotifyReadable calls this method whenever the watch for our file triggers.
  void FileNotified(const char *filename) {
    CHECK_NE(watch_, -1);
    LOG(DEBUG, "got a notification for %s\n", filename_.c_str());

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

  // To make sure that Init gets called exactly once.
  static ::aos::Once<void> init_once;

  const std::string filename_;
  const std::function<void(void *)> callback_;
  void *const value_;
  const bool create_;
  std::string check_filename_;

  // The watch descriptor or -1 if we don't have one any more.
  int watch_;
  // The watch that we still have to take out of the map once we get the
  // IN_IGNORED or -1.
  int watch_to_remove_ = -1;

  // Map from watch IDs to instances of this class.
  // <https://patchwork.kernel.org/patch/73192/> ("inotify: do not reuse watch
  // descriptors") says they won't get reused, but that shouldn't be counted on
  // because we might have a modified/different version/whatever kernel.
  static std::map<int, FileWatch *> watchers;
  // The inotify(7) file descriptor.
  static int notify_fd;

  DISALLOW_COPY_AND_ASSIGN(FileWatch);
};
::aos::Once<void> FileWatch::init_once(FileWatch::Init);
std::map<int, FileWatch *> FileWatch::watchers;
int FileWatch::notify_fd;

// Runs the given command and returns its first line of output (not including
// the \n). LOG(FATAL)s if the command has an exit status other than 0 or does
// not print out an entire line.
std::string RunCommand(std::string command) {
  // popen(3) might fail and not set it.
  errno = 0;
  FILE *pipe = popen(command.c_str(), "r");
  if (pipe == NULL) {
    PLOG(FATAL, "popen(\"%s\", \"r\") failed", command.c_str());
  }

  // result_size is how many bytes result is currently allocated to.
  size_t result_size = 128, read = 0;
  unique_c_ptr<char> result(static_cast<char *>(malloc(result_size)));
  while (true) {
    // If we filled up the buffer, then realloc(3) it bigger.
    if (read == result_size) {
      result_size *= 2;
      void *new_result = realloc(result.get(), result_size);
      if (new_result == NULL) {
        PLOG(FATAL, "realloc(%p, %zd) failed", result.get(), result_size);
      } else {
        result.release();
        result = unique_c_ptr<char>(static_cast<char *>(new_result));
      }
    }

    size_t ret = fread(result.get() + read, 1, result_size - read, pipe);
    // If the read didn't fill up the whole buffer, check to see if it was
    // because of an error.
    if (ret < result_size - read) {
      if (ferror(pipe)) {
        PLOG(FATAL, "couldn't finish reading output of \"%s\"\n",
             command.c_str());
      }
    }
    read += ret;
    if (read > 0 && result.get()[read - 1] == '\n') {
      break;
    }

    if (feof(pipe)) {
      LOG(FATAL, "`%s` failed. didn't print a whole line\n", command.c_str());
    }
  }

  // Get rid of the first \n and anything after it.
  *strchrnul(result.get(), '\n') = '\0';

  int child_status = pclose(pipe);
  if (child_status == -1) {
    PLOG(FATAL, "pclose(%p) failed", pipe);
  }

  if (child_status != 0) {
    LOG(FATAL, "`%s` failed. return %d\n", command.c_str(), child_status);
  }

  return std::string(result.get());
}

// Will call callback(arg) after time.
void Timeout(monotonic_clock::duration time,
             void (*callback)(int, short, void *), void *arg) {
  EventUniquePtr timeout(evtimer_new(libevent_base.get(), callback, arg));
  struct timeval time_timeval;
  {
    ::std::chrono::seconds sec =
        ::std::chrono::duration_cast<::std::chrono::seconds>(time);
    ::std::chrono::microseconds usec =
        ::std::chrono::duration_cast<::std::chrono::microseconds>(time - sec);
    time_timeval.tv_sec = sec.count();
    time_timeval.tv_usec = usec.count();
  }
  if (evtimer_add(timeout.release(), &time_timeval) != 0) {
    LOG(FATAL, "evtimer_add(%p, %p) failed\n", timeout.release(),
        &time_timeval);
  }
}

class Child;
// This is where all of the Child instances except core live.
std::vector<unique_ptr<Child>> children;
// A global place to hold on to which child is core.
unique_ptr<Child> core;

// Represents a child process. It will take care of restarting itself etc.
class Child {
 public:
  // command is the (space-separated) command to run and its arguments.
  Child(const std::string &command) : pid_(-1),
        stat_at_start_valid_(false) {
    if (!restart_timeout) {
      restart_timeout = EventUniquePtr(
          evtimer_new(libevent_base.get(), StaticDoRestart, nullptr));
    }
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

    original_binary_ = RunCommand("which " + args_[0]);
    binary_ = original_binary_ + ".stm";

    watcher_ = unique_ptr<FileWatch>(
        new FileWatch(original_binary_, StaticFileModified, this));

    Start();
  }

  pid_t pid() { return pid_; }

  // This gets called whenever the actual process dies and should (probably) be
  // restarted.
  void ProcessDied() {
    pid_ = -1;
    restarts_.push(monotonic_clock::now());
    if (restarts_.size() > kMaxRestartsNumber) {
      monotonic_clock::time_point oldest = restarts_.front();
      restarts_.pop();
      if (monotonic_clock::now() <= kMaxRestartsTime + oldest) {
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
  static constexpr chrono::nanoseconds kProcessDieTime = chrono::seconds(2);

  // How long to wait after the file is modified to restart it.
  // This is important because some programs like modifying the binaries by
  // writing them in little bits, which results in attempting to start partial
  // binaries without this.
  static constexpr chrono::nanoseconds kRestartWaitTime =
      chrono::milliseconds(1500);

  // Only kMaxRestartsNumber restarts will be allowed in kMaxRestartsTime.
  static constexpr chrono::nanoseconds kMaxRestartsTime = chrono::seconds(4);
  static const size_t kMaxRestartsNumber = 3;
  // How long to wait if it gets restarted too many times.
  static constexpr chrono::nanoseconds kResumeWait = chrono::seconds(5);

  static void StaticFileModified(void *self) {
    static_cast<Child *>(self)->FileModified();
  }

  void FileModified() {
    LOG(DEBUG, "file for %s modified\n", name());
    struct timeval restart_time_timeval;
    {
      ::std::chrono::seconds sec =
          ::std::chrono::duration_cast<::std::chrono::seconds>(
              kRestartWaitTime);
      ::std::chrono::microseconds usec =
          ::std::chrono::duration_cast<::std::chrono::microseconds>(
              kRestartWaitTime - sec);
      restart_time_timeval.tv_sec = sec.count();
      restart_time_timeval.tv_usec = usec.count();
    }
    // This will reset the timeout again if it hasn't run yet.
    if (evtimer_add(restart_timeout.get(), &restart_time_timeval) != 0) {
      LOG(FATAL, "evtimer_add(%p, %p) failed\n", restart_timeout.get(),
          &restart_time_timeval);
    }
    waiting_to_restart.insert(this);
  }

  static void StaticDoRestart(int, short, void *) {
    LOG(DEBUG, "restarting everything that needs it\n");
    if (waiting_to_restart.find(core.get()) != waiting_to_restart.end()) {
      core->DoRestart();
      waiting_to_restart.erase(core.get());
    }
    for (auto c : waiting_to_restart) {
      c->DoRestart();
    }
    waiting_to_restart.clear();
  }

  // Called after somebody else has finished modifying the file.
  void DoRestart() {
    fprintf(stderr, "DoRestart(%s)\n", binary_.c_str());
    if (stat_at_start_valid_) {
      struct stat current_stat;
      if (stat(original_binary_.c_str(), &current_stat) == -1) {
        PLOG(FATAL, "stat(%s, %p) failed",
             original_binary_.c_str(), &current_stat);
      }
      if (current_stat.st_mtime == stat_at_start_.st_mtime) {
        LOG(DEBUG, "ignoring trigger for %s because mtime didn't change\n",
            name());
        return;
      }
    }

    if (this == core.get()) {
      fprintf(stderr, "Restarting core -> exiting now.\n");
      exit(0);
    }
    if (pid_ != -1) {
      LOG(DEBUG, "sending SIGTERM to child %d to restart it\n", pid_);
      if (kill(pid_, SIGTERM) == -1) {
        PLOG(WARNING, "kill(%d, SIGTERM) failed", pid_);
      }
      CheckDiedStatus *status = new CheckDiedStatus();
      status->self = this;
      status->old_pid = pid_;
      Timeout(kProcessDieTime, StaticCheckDied, status);
    } else {
      LOG(WARNING, "%s restart attempted but not running\n", name());
    }
  }

  static void StaticCheckDied(int, short, void *status_in) {
    CheckDiedStatus *status = static_cast<CheckDiedStatus *>(status_in);
    status->self->CheckDied(status->old_pid);
    delete status;
  }

  // Checks to see if the child using the PID old_pid is still running.
  void CheckDied(pid_t old_pid) {
    if (pid_ == old_pid) {
      LOG(WARNING, "child %d refused to die\n", old_pid);
      if (kill(old_pid, SIGKILL) == -1) {
        PLOG(WARNING, "kill(%d, SIGKILL) failed", old_pid);
      }
    }
  }

  static void StaticStart(int, short, void *self) {
    static_cast<Child *>(self)->Start();
  }

  // Actually starts the child.
  void Start() {
    if (pid_ != -1) {
      LOG(WARNING, "calling Start() but already have child %d running\n",
          pid_);
      if (kill(pid_, SIGKILL) == -1) {
        PLOG(WARNING, "kill(%d, SIGKILL) failed", pid_);
        return;
      }
      pid_ = -1;
    }

    // Remove the name that we run from (ie from a previous execution) and then
    // hard link the real filename to it.
    if (unlink(binary_.c_str()) != 0 && errno != ENOENT) {
      PLOG(FATAL, "removing %s failed", binary_.c_str());
    }
    if (link(original_binary_.c_str(), binary_.c_str()) != 0) {
      PLOG(FATAL, "link('%s', '%s') failed",
           original_binary_.c_str(), binary_.c_str());
    }

    if (stat(original_binary_.c_str(), &stat_at_start_) == -1) {
      PLOG(FATAL, "stat(%s, %p) failed",
           original_binary_.c_str(), &stat_at_start_);
    }
    stat_at_start_valid_ = true;

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
      PLOG(FATAL, "execv(%s, %p) failed", binary_.c_str(), argv);
      _exit(EXIT_FAILURE);
    }
    if (pid_ == -1) {
      PLOG(FATAL, "forking to run \"%s\" failed", binary_.c_str());
    }
    LOG(DEBUG, "started \"%s\" successfully\n", binary_.c_str());
  }

  // A history of the times that this process has been restarted.
  std::queue<monotonic_clock::time_point,
             std::list<monotonic_clock::time_point>> restarts_;

  // The currently running child's PID or NULL.
  pid_t pid_;

  // All of the arguments (including the name of the binary).
  std::deque<std::string> args_;

  // The name of the real binary that we were told to run.
  std::string original_binary_;
  // The name of the file that we're actually running.
  std::string binary_;

  // Watches original_binary_.
  unique_ptr<FileWatch> watcher_;

  // Captured from the original file when we most recently started a new child
  // process. Used to see if it actually changes or not.
  struct stat stat_at_start_;
  bool stat_at_start_valid_;

  // An event that restarts after kRestartWaitTime.
  static EventUniquePtr restart_timeout;

  // The set of children waiting to be restarted once all modifications stop.
  static ::std::set<Child *> waiting_to_restart;

  DISALLOW_COPY_AND_ASSIGN(Child);
};

constexpr chrono::nanoseconds Child::kProcessDieTime;
constexpr chrono::nanoseconds Child::kRestartWaitTime;
constexpr chrono::nanoseconds Child::kMaxRestartsTime;
constexpr chrono::nanoseconds Child::kResumeWait;

EventUniquePtr Child::restart_timeout;
::std::set<Child *> Child::waiting_to_restart;

// Kills off the entire process group (including ourself).
void KillChildren(bool try_nice) {
  if (try_nice) {
    static constexpr int kNiceStopSignal = SIGTERM;
    static constexpr auto kNiceWaitTime = chrono::seconds(1);

    // Make sure that we don't just nicely stop ourself...
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, kNiceStopSignal);
    sigprocmask(SIG_BLOCK, &mask, NULL);

    kill(-getpid(), kNiceStopSignal);

    fflush(NULL);
    ::std::this_thread::sleep_for(kNiceWaitTime);
  }

  // Send SIGKILL to our whole process group, which will forcibly terminate any
  // of them that are still running (us for sure, maybe more too).
  kill(-getpid(), SIGKILL);
}

void ExitHandler() {
  KillChildren(true);
}

void KillChildrenSignalHandler(int signum) {
  // If we get SIGSEGV or some other random signal who knows what's happening
  // and we should just kill everybody immediately.
  // This is a list of all of the signals that mean some form of "nicely stop".
  KillChildren(signum == SIGHUP || signum == SIGINT || signum == SIGQUIT ||
               signum == SIGABRT || signum == SIGPIPE || signum == SIGTERM ||
               signum == SIGXCPU);
}

// Returns the currently running child with PID pid or an empty unique_ptr.
const unique_ptr<Child> &FindChild(pid_t pid) {
  for (auto it = children.begin(); it != children.end(); ++it) {
    if (pid == (*it)->pid()) {
      return *it;
    }
  }

  if (pid == core->pid()) {
    return core;
  }

  static const unique_ptr<Child> kNothing;
  return kNothing;
}

// Gets set up as a libevent handler for SIGCHLD.
// Handles calling Child::ProcessDied() on the appropriate one.
void SigCHLDReceived(int /*fd*/, short /*events*/, void *) {
  // In a while loop in case we miss any SIGCHLDs.
  while (true) {
    siginfo_t infop;
    infop.si_pid = 0;
    if (waitid(P_ALL, 0, &infop, WEXITED | WSTOPPED | WNOHANG) != 0) {
      PLOG(WARNING, "waitid failed");
      continue;
    }
    // If there are no more child process deaths to process.
    if (infop.si_pid == 0) {
      return;
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
              pid, child->name(), status, aos_strsignal(status));
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
      LOG(WARNING, "couldn't find a Child for pid %d\n", pid);
      return;
    }

    if (child == core) {
      LOG(FATAL, "core died\n");
    }
    child->ProcessDied();
  }
}

// This is used for communicating the name of the file to read processes to
// start from main to Run.
const char *child_list_file;

void Run(void *watch);
void Main() {
  logging::Init();

  // Set UID to 0 so we can run things as root down below. Since the starter
  // program on the roborio runs starter.sh under "lvuser", it will continuously
  // fail due to lack of permissions if we do not manually set the UID to admin.
#ifdef AOS_ARCHITECTURE_arm_frc
  if (setuid(0) != 0) {
    PLOG(FATAL, "setuid(0) failed");
  }
#endif

  if (setpgid(0 /*self*/, 0 /*make PGID the same as PID*/) != 0) {
    PLOG(FATAL, "setpgid(0, 0) failed");
  }

  // Make sure that we kill all children when we exit.
  atexit(ExitHandler);
  // Do it on some signals too (ones that we otherwise tend to receive and then
  // leave all of our children going).
  signal(SIGHUP, KillChildrenSignalHandler);
  signal(SIGINT, KillChildrenSignalHandler);
  signal(SIGQUIT, KillChildrenSignalHandler);
  signal(SIGILL, KillChildrenSignalHandler);
  signal(SIGABRT, KillChildrenSignalHandler);
  signal(SIGFPE, KillChildrenSignalHandler);
  signal(SIGSEGV, KillChildrenSignalHandler);
  signal(SIGPIPE, KillChildrenSignalHandler);
  signal(SIGTERM, KillChildrenSignalHandler);
  signal(SIGBUS, KillChildrenSignalHandler);
  signal(SIGXCPU, KillChildrenSignalHandler);

#ifdef AOS_ARCHITECTURE_arm_frc
  // Just allow overcommit memory like usual. Various processes map memory they
  // will never use, and the roboRIO doesn't have enough RAM to handle it.
  // This is in here instead of starter.sh because starter.sh doesn't run with
  // permissions on a roboRIO.
  CHECK(system("echo 0 > /proc/sys/vm/overcommit_memory") == 0);
#endif
  
  libevent_base = EventBaseUniquePtr(event_base_new());

  std::string core_touch_file = "/tmp/starter.";
  core_touch_file += std::to_string(static_cast<intmax_t>(getpid()));
  core_touch_file += ".core_touch_file";
  const int result =
      ::aos::util::RunCommand(("touch '" + core_touch_file + "'").c_str());
  if (result == -1) {
    PLOG(FATAL, "running `touch '%s'` failed\n", core_touch_file.c_str());
  } else if (!WIFEXITED(result) || WEXITSTATUS(result) != 0) {
    LOG(FATAL, "`touch '%s'` gave result %x\n", core_touch_file.c_str(),
        result);
  }
  FileWatch core_touch_file_watch(core_touch_file, Run, NULL);
  core = unique_ptr<Child>(
      new Child("core " + core_touch_file));

  FILE *pid_file = fopen("/tmp/starter.pid", "w");
  if (pid_file == NULL) {
    PLOG(FATAL, "fopen(\"/tmp/starter.pid\", \"w\") failed");
  } else {
    if (fprintf(pid_file, "%d", core->pid()) == -1) {
      PLOG(WARNING, "fprintf(%p, \"%%d\", %d) failed",
           pid_file, core->pid());
    }
    fclose(pid_file);
  }

  LOG(INFO, "waiting for %s to appear\n", core_touch_file.c_str());

  event_base_dispatch(libevent_base.get());
  LOG(FATAL, "event_base_dispatch(%p) returned\n", libevent_base.get());
}

// This is the callback for when core creates the file indicating that it has
// started.
void Run(void *watch) {
  // Make it so it doesn't keep on seeing random changes in /tmp.
  static_cast<FileWatch *>(watch)->RemoveWatch();

  // It's safe now because core is up.
  aos::InitNRT();

  std::ifstream list_file(child_list_file);
  
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
                                   SigCHLDReceived, NULL));
  event_add(sigchld.release(), NULL);
}

const char *kArgsHelp = "[OPTION]... START_LIST\n"
    "Start all of the robot code binaries in START_LIST.\n"
    "\n"
    "START_LIST is the file to read binaries (looked up on PATH) to run.\n"
    "  --help        display this help and exit\n";
void PrintHelp() {
  fprintf(stderr, "Usage: %s %s", program_invocation_name, kArgsHelp);
}

}  // namespace starter
}  // namespace aos

int main(int argc, char *argv[]) {
  if (argc != 2) {
    aos::starter::PrintHelp();
    exit(EXIT_FAILURE);
  }
  if (strcmp(argv[1], "--help") == 0) {
    aos::starter::PrintHelp();
    exit(EXIT_SUCCESS);
  }

  aos::starter::child_list_file = argv[1];

  aos::starter::Main();
}
