#ifndef AOS_STARTER_SUBPROCESS_H_
#define AOS_STARTER_SUBPROCESS_H_

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/starter/starter_generated.h"
#include "aos/starter/starter_rpc_generated.h"
#include "aos/util/scoped_pipe.h"
#include "aos/util/top.h"

namespace aos::starter {

// Registers a signalfd listener with the given event loop and calls callback
// whenever a signal is received.
class SignalListener {
 public:
  SignalListener(aos::ShmEventLoop *loop,
                 std::function<void(signalfd_siginfo)> callback);
  SignalListener(aos::internal::EPoll *epoll,
                 std::function<void(signalfd_siginfo)> callback);
  SignalListener(aos::ShmEventLoop *loop,
                 std::function<void(signalfd_siginfo)> callback,
                 std::initializer_list<unsigned int> signals);
  SignalListener(aos::internal::EPoll *epoll,
                 std::function<void(signalfd_siginfo)> callback,
                 std::initializer_list<unsigned int> signals);

  ~SignalListener();

 private:
  aos::internal::EPoll *epoll_;
  std::function<void(signalfd_siginfo)> callback_;
  aos::ipc_lib::SignalFd signalfd_;

  DISALLOW_COPY_AND_ASSIGN(SignalListener);
};

// Class to use the V1 cgroup API to limit memory usage.
class MemoryCGroup {
 public:
  MemoryCGroup(std::string_view name);
  ~MemoryCGroup();

  // Adds a thread ID to be managed by the cgroup.
  void AddTid(pid_t pid = 0);

  // Sets the provided limit to the provided value.
  void SetLimit(std::string_view limit_name, uint64_t limit_value);

 private:
  std::string cgroup_;
};

// Manages a running process, allowing starting and stopping, and restarting
// automatically.
class Application {
 public:
  enum class QuietLogging { kYes, kNo };
  Application(const aos::Application *application, aos::EventLoop *event_loop,
              std::function<void()> on_change,
              QuietLogging quiet_flag = QuietLogging::kNo);

  // executable_name is the actual executable path.
  // When sudo is not used, name is used as argv[0] when exec'ing
  // executable_name. When sudo is used it's not possible to pass in a
  // distinct argv[0].
  Application(std::string_view name, std::string_view executable_name,
              aos::EventLoop *event_loop, std::function<void()> on_change,
              QuietLogging quiet_flag = QuietLogging::kNo);

  flatbuffers::Offset<aos::starter::ApplicationStatus> PopulateStatus(
      flatbuffers::FlatBufferBuilder *builder, util::Top *top);
  aos::starter::State status() const { return status_; };

  // Returns the last pid of this process. -1 if not started yet.
  pid_t get_pid() const { return pid_; }

  // Handles a SIGCHLD signal received by the parent. Does nothing if this
  // process was not the target. Returns true if this Application should be
  // removed.
  bool MaybeHandleSignal();
  void DisableChildDeathPolling() { child_status_handler_->Disable(); }

  // Handles a command. May do nothing if application is already in the desired
  // state.
  void HandleCommand(aos::starter::Command cmd);

  void Start() { HandleCommand(aos::starter::Command::START); }

  void Stop() { HandleCommand(aos::starter::Command::STOP); }

  void Terminate();

  // Adds a callback which gets notified when the application changes state.
  // This is in addition to any existing callbacks and doesn't replace any of
  // them.
  void AddOnChange(std::function<void()> fn) {
    on_change_.emplace_back(std::move(fn));
  }

  void set_args(std::vector<std::string> args);
  void set_capture_stdout(bool capture);
  void set_capture_stderr(bool capture);
  void set_run_as_sudo(bool value) { run_as_sudo_ = value; }

  bool autostart() const { return autostart_; }

  bool autorestart() const { return autorestart_; }

  const std::string &GetStdout();
  const std::string &GetStderr();
  std::optional<int> exit_code() const { return exit_code_; }

  // Sets the memory limit for the application to the provided limit.
  void SetMemoryLimit(size_t limit) {
    if (!memory_cgroup_) {
      memory_cgroup_ = std::make_unique<MemoryCGroup>(name_);
    }
    memory_cgroup_->SetLimit("memory.limit_in_bytes", limit);
  }

 private:
  typedef aos::util::ScopedPipe::PipePair PipePair;

  static constexpr const char *const kSudo{"sudo"};

  void set_args(
      const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>
          &args);

  void DoStart();

  void DoStop(bool restart);

  void QueueStart();

  void OnChange();

  // Copy flatbuffer vector of strings to vector of std::string.
  static std::vector<std::string> FbsVectorToVector(
      const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>> &v);

  static std::optional<uid_t> FindUid(const char *name);
  static std::optional<gid_t> FindPrimaryGidForUser(const char *name);

  void FetchOutputs();

  // Provides an std::vector of the args (such that CArgs().data() ends up being
  // suitable to pass to execve()).
  // The points are invalidated when args_ changes (e.g., due to a set_args
  // call).
  std::vector<char *> CArgs();

  // Next unique id for all applications
  static inline uint64_t next_id_ = 0;

  std::string name_;
  std::string path_;
  std::vector<std::string> args_;
  std::string user_name_;
  std::optional<uid_t> user_;
  std::optional<gid_t> group_;
  bool run_as_sudo_ = false;

  bool capture_stdout_ = false;
  PipePair stdout_pipes_;
  std::string stdout_;
  bool capture_stderr_ = false;
  PipePair stderr_pipes_;
  std::string stderr_;

  pid_t pid_ = -1;
  PipePair status_pipes_;
  uint64_t id_ = 0;
  std::optional<int> exit_code_;
  aos::monotonic_clock::time_point start_time_, exit_time_;
  bool queue_restart_ = false;
  bool terminating_ = false;
  bool autostart_ = false;
  bool autorestart_ = false;

  aos::starter::State status_ = aos::starter::State::STOPPED;
  aos::starter::LastStopReason stop_reason_ =
      aos::starter::LastStopReason::STOP_REQUESTED;

  aos::EventLoop *event_loop_;
  aos::TimerHandler *start_timer_, *restart_timer_, *stop_timer_, *pipe_timer_,
      *child_status_handler_;

  std::vector<std::function<void()>> on_change_;

  std::unique_ptr<MemoryCGroup> memory_cgroup_;

  QuietLogging quiet_flag_ = QuietLogging::kNo;

  DISALLOW_COPY_AND_ASSIGN(Application);
};

}  // namespace aos::starter
#endif  // AOS_STARTER_SUBPROCESS_H_
