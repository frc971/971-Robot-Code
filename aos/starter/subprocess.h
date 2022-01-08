#ifndef AOS_STARTER_SUBPROCESS_H_
#define AOS_STARTER_SUBPROCESS_H_

#include <string>
#include <vector>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/starter/starter_generated.h"
#include "aos/starter/starter_rpc_generated.h"
#include "aos/util/scoped_pipe.h"

namespace aos::starter {

// Registers a signalfd listener with the given event loop and calls callback
// whenever a signal is received.
class SignalListener {
 public:
  SignalListener(aos::ShmEventLoop *loop,
                 std::function<void(signalfd_siginfo)> callback);
  SignalListener(aos::ShmEventLoop *loop,
                 std::function<void(signalfd_siginfo)> callback,
                 std::initializer_list<unsigned int> signals);

  ~SignalListener();

 private:
  aos::ShmEventLoop *loop_;
  std::function<void(signalfd_siginfo)> callback_;
  aos::ipc_lib::SignalFd signalfd_;

  DISALLOW_COPY_AND_ASSIGN(SignalListener);
};

// Manages a running process, allowing starting and stopping, and restarting
// automatically.
class Application {
 public:
  Application(const aos::Application *application,
              aos::EventLoop *event_loop, std::function<void()> on_change);

  flatbuffers::Offset<aos::starter::ApplicationStatus> PopulateStatus(
      flatbuffers::FlatBufferBuilder *builder);

  // Returns the last pid of this process. -1 if not started yet.
  pid_t get_pid() const { return pid_; }

  // Handles a SIGCHLD signal received by the parent. Does nothing if this
  // process was not the target. Returns true if this Application should be
  // removed.
  bool MaybeHandleSignal();

  // Handles a command. May do nothing if application is already in the desired
  // state.
  void HandleCommand(aos::starter::Command cmd);

  void Start() { HandleCommand(aos::starter::Command::START); }

  void Stop() { HandleCommand(aos::starter::Command::STOP); }

  void Terminate();

  void set_args(
      const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>
          &args);

  bool autostart() const { return autostart_; }

  bool autorestart() const { return autorestart_; }

 private:
  void DoStart();

  void DoStop(bool restart);

  void QueueStart();

  // Copy flatbuffer vector of strings to vector of std::string.
  static std::vector<std::string> FbsVectorToVector(
      const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>> &v);

  static std::optional<uid_t> FindUid(const char *name);
  static std::optional<gid_t> FindPrimaryGidForUser(const char *name);

  // Next unique id for all applications
  static inline uint64_t next_id_ = 0;

  std::string name_;
  std::string path_;
  std::vector<char *> args_;
  std::string user_name_;
  std::optional<uid_t> user_;
  std::optional<gid_t> group_;

  pid_t pid_ = -1;
  util::ScopedPipe::ScopedReadPipe read_pipe_;
  util::ScopedPipe::ScopedWritePipe write_pipe_;
  uint64_t id_;
  int exit_code_ = 0;
  aos::monotonic_clock::time_point start_time_, exit_time_;
  bool queue_restart_ = false;
  bool terminating_ = false;
  bool autostart_ = true;
  bool autorestart_ = true;

  aos::starter::State status_ = aos::starter::State::STOPPED;
  aos::starter::LastStopReason stop_reason_ =
      aos::starter::LastStopReason::STOP_REQUESTED;

  aos::EventLoop *event_loop_;
  aos::TimerHandler *start_timer_, *restart_timer_, *stop_timer_;

  std::function<void()> on_change_;

  DISALLOW_COPY_AND_ASSIGN(Application);
};

}  // namespace aos::starter
#endif // AOS_STARTER_SUBPROCESS_H_
